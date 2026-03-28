#include "processing_pipeline.h"

#include <zephyr/logging/log.h>
#include <memory>
#include "sensor_source_stage.h"
#include <queue>
LOG_MODULE_REGISTER(processing_pipeline, LOG_LEVEL_DBG);

ProcessingPipeline::ProcessingPipeline() {
    // Initialize the source_map
    this->source_map = std::map<uint8_t, std::vector<size_t>>{};
    this->stages = std::vector<PipelineNode>{};
}

void ProcessingPipeline::add_source(const char *name, std::unique_ptr<SensorSourceStage> source) {
    uint8_t sensor_id = source->get_sensor_id();

    // check if source_map already has an entry for this sensor ID
    if (this->source_map.find(sensor_id) == this->source_map.end()) {
        LOG_DBG("Registering new source ID %d", sensor_id);
        this->source_map[sensor_id] = {};
    }

    this->source_map[sensor_id].push_back(this->stages.size());
    this->stages.push_back({name, std::move(source), {}, {}});
    LOG_DBG("Added source %s for sensor ID %d", name, sensor_id);
}
    
void ProcessingPipeline::add_stage(const char *name, std::unique_ptr<SensorProcessingStage> stage) {
    stages.push_back({name, std::move(stage), {}, {}});
    LOG_DBG("Added stage %s", name);
}

//TODO: handle errors
void ProcessingPipeline::connect(size_t src, size_t dst, size_t dst_port) {
    if (src < stages.size() && dst < stages.size()) {
        stages[dst].inputs.push_back({src, dst, dst_port});
        stages[src].outputs.push_back({src, dst, dst_port});
    }
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
    
    std::queue<size_t> q;
    
    // Helper: enqueue all direct children of a node
    auto enqueue_children = [&](size_t idx) {
        for (const Edge& e : stages[idx].outputs) {
            // e.dst is the child node index
            if (e.dst < stages.size()) {
                q.push(e.dst);
            }
        }
    };

    // The start node is already seeded (has_output=true) by inject().
    // We begin by enqueueing its children.
    enqueue_children(start_idx);

    while (!q.empty()) {
        size_t i = q.front();
        q.pop();

        PipelineNode& node = stages[i];
        const size_t in_count = node.stage->get_in_ports();

        // SOURCE-LIKE NODES (0 inputs): don't try to resolve inputs
        if (in_count == 0) {
            // If this node already has output, just propagate to its children.
            // (If you want, you could call node.stage->process(nullptr, &node.output) here.)
            if (node.has_output) {
                enqueue_children(i);
            } else {
                // Not seeded yet; nothing to do.
                LOG_DBG("Node %s is a 0-input node without output; skipping", node.name);
            }
            continue;
        }

        // Sanity: wiring must match declared ports
        if (node.inputs.size() != in_count) {
            LOG_ERR("Node %s: expected %zu inputs, but have %zu",
                    node.name, in_count, node.inputs.size());
            continue;  // or return -EINVAL;
        }

        // Resolve inputs from upstream nodes
        std::vector<const sensor_data*> inputs(in_count);
        bool ready = true;
        for (size_t p = 0; p < in_count; ++p) {
            const Edge& e = node.inputs[p];
            if (e.src >= stages.size() || !stages[e.src].has_output) {
                ready = false;
                break;
            }
            inputs[p] = &stages[e.src].output;
        }

        if (!ready) {
            // Upstreams not ready yet; skip for now.
            // (Optional: you can re-enqueue i later if you keep a scheduler.)
            LOG_DBG("Node %s not ready; skipping", node.name);
            continue;
        }

        // Process the node
        int rc = node.stage->process(inputs.data(), &node.output);
        if (rc < 0) {
            LOG_ERR("Error processing node %s: %d", node.name, rc);
            // Error from stage
            return rc;
        } else if (rc > 0) {
            node.has_output = false;
            continue;
        } else {
            // Valid result produced
            node.has_output = true;
            enqueue_children(i);
        }
    }

    return 0;
}

const struct sensor_data& ProcessingPipeline::get_output(size_t node_index) const {
    return stages[node_index].output;
}
