#include "processing_pipeline.h"

#include <algorithm>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(processing_pipeline, LOG_LEVEL_DBG);

ProcessingPipeline::ProcessingPipeline() {
    this->source_map = std::map<uint8_t, std::vector<size_t>>{};
    this->stages = std::vector<PipelineNode>{};
    this->queued_generation = std::vector<uint32_t>{};
}

void ProcessingPipeline::add_source(const char *name, std::unique_ptr<SensorSourceStage> source) {
    const size_t node_index = this->stages.size();
    uint8_t sensor_id = source->get_sensor_id();

    // check if source_map already has an entry for this sensor ID
    if (this->source_map.find(sensor_id) == this->source_map.end()) {
        LOG_DBG("Registering new source ID %d", sensor_id);
        this->source_map[sensor_id] = {};
    }

    this->source_map[sensor_id].push_back(node_index);
    this->stages.push_back({name, std::move(source), {}, {}});
    this->queued_generation.push_back(0);
    LOG_DBG("Added source %s for sensor ID %d", name, sensor_id);
}
    
void ProcessingPipeline::add_stage(const char *name, std::unique_ptr<SensorProcessingStage> stage) {
    const size_t in_ports = stage->get_in_ports();
    const size_t node_index = stages.size();
    std::vector<Edge> inputs;
    inputs.reserve(in_ports);
    for (size_t port = 0; port < in_ports; ++port) {
        inputs.push_back({SIZE_MAX, node_index, port});
    }

    stages.push_back({name, std::move(stage), std::move(inputs), {}});
    queued_generation.push_back(0);
    LOG_DBG("Added stage %s", name);
}

//TODO: handle errors
void ProcessingPipeline::connect(size_t src, size_t dst, size_t dst_port) {
    if (src < stages.size() && dst < stages.size()) {
        if (dst_port >= stages[dst].stage->get_in_ports()) {
            LOG_ERR("Invalid destination port %zu for node %s", dst_port, stages[dst].name);
            return;
        }

        if (stages[dst].inputs[dst_port].src != SIZE_MAX) {
            LOG_ERR("Node %s input port %zu already connected", stages[dst].name, dst_port);
            return;
        }

        stages[dst].inputs[dst_port] = {src, dst, dst_port};
        stages[src].outputs.push_back({src, dst, dst_port});
        return;
    }

    LOG_ERR("Invalid edge connection src=%zu dst=%zu port=%zu", src, dst, dst_port);
}

void ProcessingPipeline::connect(const char* src, const char *dest, size_t dst_port) {
    size_t src_idx = stages.size();
    size_t dst_idx = stages.size();
    for (size_t i = 0; i < stages.size(); ++i) {
        if (strcmp(stages[i].name, src) == 0) {
            src_idx = i;
        }
        if (strcmp(stages[i].name, dest) == 0) {
            dst_idx = i;
        }
    }
    this->connect(src_idx, dst_idx, dst_port);
}

int ProcessingPipeline::inject(const struct sensor_data& sample) {
    auto it = source_map.find(sample.id);
    if (it == source_map.end() || it->second.empty()) {
        return -EINVAL; // no matching source
    }

    for (size_t src_idx : it->second) {
        // Seed source output
        stages[src_idx].output     = sample;
        stages[src_idx].has_output = true;

        // Propagate from this source
        int rc = run_from(src_idx);
        if (rc < 0) return rc;
    }
    return 0;
}

int ProcessingPipeline::run() {
    return run_from(0);
}

int ProcessingPipeline::run_from(size_t start_idx) {
    if (start_idx >= stages.size()) return -EINVAL;

    std::queue<size_t> ready_queue;

    if (++run_generation == 0) {
        run_generation = 1;
        std::fill(queued_generation.begin(), queued_generation.end(), 0);
    }

    auto enqueue_children_if_ready = [&](size_t idx) {
        for (const Edge& e : stages[idx].outputs) {
            if (e.dst >= stages.size()) {
                LOG_ERR("Node %s has invalid child index %zu", stages[idx].name, e.dst);
                continue;
            }

            enqueue_if_ready(e.dst, ready_queue);
        }
    };

    const int source_rc = materialize_autonomous_sources(ready_queue);
    if (source_rc < 0) {
        return source_rc;
    }

    enqueue_children_if_ready(start_idx);

    while (!ready_queue.empty()) {
        size_t i = ready_queue.front();
        ready_queue.pop();
        queued_generation[i] = 0;

        PipelineNode& node = stages[i];
        const size_t in_count = node.stage->get_in_ports();

        if (in_count == 0) {
            if (node.has_output) {
                enqueue_children_if_ready(i);
            }
            continue;
        }

        if (!is_node_ready(i)) {
            LOG_DBG("Node %s is no longer ready; skipping", node.name);
            continue;
        }

        std::vector<const sensor_data*> inputs(in_count, nullptr);
        for (size_t port = 0; port < in_count; ++port) {
            inputs[port] = &stages[node.inputs[port].src].output;
        }

        int rc = node.stage->process(inputs.data(), &node.output);
        if (rc < 0) {
            LOG_ERR("Error processing node %s: %d", node.name, rc);
            return rc;
        }

        if (rc > 0) {
            node.has_output = false;
            continue;
        }

        node.has_output = true;
        enqueue_children_if_ready(i);
    }

    return 0;
}

const struct sensor_data& ProcessingPipeline::get_output(size_t node_index) const {
    return stages[node_index].output;
}

int ProcessingPipeline::materialize_autonomous_sources(std::queue<size_t>& ready_queue) {
    for (size_t i = 0; i < stages.size(); ++i) {
        PipelineNode& node = stages[i];
        if (node.stage->get_in_ports() != 0 || !node.stage->is_autonomous_source()) {
            continue;
        }

        const int rc = node.stage->process(nullptr, &node.output);
        if (rc < 0) {
            LOG_ERR("Error processing autonomous source %s: %d", node.name, rc);
            return rc;
        }

        node.has_output = (rc == 0);
        if (!node.has_output) {
            continue;
        }

        for (const Edge& e : node.outputs) {
            if (e.dst >= stages.size()) {
                LOG_ERR("Node %s has invalid child index %zu", node.name, e.dst);
                continue;
            }

            enqueue_if_ready(e.dst, ready_queue);
        }
    }

    return 0;
}

bool ProcessingPipeline::is_node_ready(size_t node_index) const {
    if (node_index >= stages.size()) {
        return false;
    }

    const PipelineNode& node = stages[node_index];
    const size_t in_count = node.stage->get_in_ports();

    if (in_count == 0) {
        return node.has_output;
    }

    if (node.inputs.size() != in_count) {
        LOG_ERR("Node %s: expected %zu inputs, but have %zu",
                node.name, in_count, node.inputs.size());
        return false;
    }

    for (const Edge& e : node.inputs) {
        if (e.src == SIZE_MAX) {
            LOG_ERR("Node %s has an unconnected input port %zu", node.name, e.dstPort);
            return false;
        }

        if (e.src >= stages.size() || !stages[e.src].has_output) {
            return false;
        }
    }

    return true;
}

bool ProcessingPipeline::enqueue_if_ready(size_t node_index, std::queue<size_t>& ready_queue) {
    if (!is_node_ready(node_index)) {
        return false;
    }

    if (queued_generation[node_index] == run_generation) {
        return false;
    }

    queued_generation[node_index] = run_generation;
    ready_queue.push(node_index);
    return true;
}
