#ifndef _PEAK_DETECTOR_STAGE_H
#define _PEAK_DETECTOR_STAGE_H

#include "sensor_processing_stage.h"
#include "ParseType.h"
#include <math.h>   // isnan
#include <cstdint>

class PeakDetectorStage : public SensorProcessingStage {
public:
    // eps: small deadband for derivative to avoid chatter
    // maxOpen: capacity of internal stacks (defaults to 16)
    PeakDetectorStage(enum ParseType parse_type, float eps = 0.0f, int maxOpen = 16);
    ~PeakDetectorStage();

    // Returns:
    //  0 -> produced one output sample (peak/trough finalized, appended sign+prominence)
    //  1 -> consumed input, no output produced yet (need more data)
    // <0 -> error (e.g., EINVAL)
    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    enum ParseType parse_type;

    // Previous full input sample and derivative info
    const struct sensor_data *last_data;
    float last_dx;
    enum Dir : int8_t { RISE = 1, FLAT = 0, FALL = -1 };
    Dir last_dir;
    float eps;

    // Last turning-point values for symmetry bookkeeping
    float last_valley; bool have_valley;
    float last_peak;   bool have_peak;

    // Open-peak stack (maxima)
    struct PeakState  { float val, left_base_min,  right_run_min;  };
    // Open-trough stack (minima)
    struct TroughState{ float val, left_base_max, right_run_max;  };

    PeakState*  pstack;
    TroughState* tstack;
    int psp, tsp, maxOpen;

    float decode_sample(const struct sensor_data* sd) const;

    inline Dir deriv_dir(float dx) const {
        if (dx >  eps) return RISE;
        if (dx < -eps) return FALL;
        return FLAT;
    }

    // Push helpers with simple overflow policy (drop oldest)
    inline void push_peak(const PeakState& s) {
        if (psp < maxOpen) { pstack[psp++] = s; return; }
        // Drop oldest (shift left)
        for (int i = 1; i < maxOpen; ++i) pstack[i-1] = pstack[i];
        pstack[maxOpen-1] = s;
        psp = maxOpen;
    }
    inline void push_trough(const TroughState& s) {
        if (tsp < maxOpen) { tstack[tsp++] = s; return; }
        for (int i = 1; i < maxOpen; ++i) tstack[i-1] = tstack[i];
        tstack[maxOpen-1] = s;
        tsp = maxOpen;
    }
};

#endif // _PEAK_DETECTOR_STAGE_H