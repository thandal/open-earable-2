#ifndef _PROCESSING_PIPELINE_H
#define _PROCESSING_PIPELINE_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <map>
#include <queue>
#include <vector>
#include "sensor_processing_stage.h"
#include "sensor_source_stage.h"

struct Edge {
    size_t src;     // index of source node
    size_t dst;     // index of destination node
    size_t dstPort; // input port on the destination
};

struct PipelineNode {
    const char *name;
    std::unique_ptr<SensorProcessingStage> stage;
    std::vector<Edge> inputs;     // indexed by destination port
    std::vector<Edge> outputs;    // edges feeding out of this node
    struct sensor_data output;           // stage result (cached for children)
    bool has_output = false; // true if the stage produced a valid output
};

class ProcessingPipeline {
public:
    ProcessingPipeline();

    void add_source(const char *name, std::unique_ptr<SensorSourceStage> source);

    void add_stage(const char *name, std::unique_ptr<SensorProcessingStage> stage);
    void connect(size_t src, size_t dst, size_t dst_port = 0);
    void connect(const char *src, const char *dst, size_t dst_port = 0);

    int run();

    int inject(const struct sensor_data& sample);

    const struct sensor_data& get_output(size_t node_index) const;

private:
    std::vector<PipelineNode> stages;
    std::map<uint8_t, std::vector<size_t>> source_map;
    uint32_t run_generation = 0;
    std::vector<uint32_t> queued_generation;

    int run_from(size_t start_index);
    int materialize_autonomous_sources(std::queue<size_t>& ready_queue);
    bool is_node_ready(size_t node_index) const;
    bool enqueue_if_ready(size_t node_index, std::queue<size_t>& ready_queue);
};

#endif // _PROCESSING_PIPELINE_H
