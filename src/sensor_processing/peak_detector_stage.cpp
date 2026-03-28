#include "peak_detector_stage.h"
#include <cstring>
#include <new>
#include "processing_utils.h"

// Clamp helpers
static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// Encode a float prominence into the same numeric type as input (parse_type)
static inline void encode_prominence(ParseType t, float v, uint8_t* dst) {
    switch (t) {
        case PARSE_TYPE_INT8:   { int8_t  s = (int8_t)  lroundf(clampf(v, -128.0f, 127.0f));      memcpy(dst, &s, sizeof(s)); break; }
        case PARSE_TYPE_UINT8:  { uint8_t s = (uint8_t) lroundf(clampf(v, 0.0f, 255.0f));         memcpy(dst, &s, sizeof(s)); break; }
        case PARSE_TYPE_INT16:  { int16_t s = (int16_t) lroundf(clampf(v, -32768.0f, 32767.0f));  memcpy(dst, &s, sizeof(s)); break; }
        case PARSE_TYPE_UINT16: { uint16_t s= (uint16_t)lroundf(clampf(v, 0.0f, 65535.0f));       memcpy(dst, &s, sizeof(s)); break; }
        case PARSE_TYPE_INT32:  { int32_t s = (int32_t) lroundf(clampf(v, -2147483648.0f, 2147483647.0f)); memcpy(dst, &s, sizeof(s)); break; }
        case PARSE_TYPE_UINT32: { uint32_t s= (uint32_t) (v < 0 ? 0 : (v > 4294967295.0f ? 4294967295.0f : v)); memcpy(dst, &s, sizeof(s)); break; }
        case PARSE_TYPE_FLOAT:  { float    s = v; memcpy(dst, &s, sizeof(s)); break; }
        case PARSE_TYPE_DOUBLE: { double   s = v; memcpy(dst, &s, sizeof(s)); break; }
        default: /* fallback to float if unknown */
            { float s = v; memcpy(dst, &s, sizeof(s)); break; }
    }
}

PeakDetectorStage::PeakDetectorStage(enum ParseType parse_type, float eps, int maxOpen)
    : SensorProcessingStage(1),
      parse_type(parse_type),
      last_data(nullptr),
      last_dx(NAN),
      last_dir(FLAT),
      eps(eps),
      last_valley(NAN), have_valley(false),
      last_peak(NAN),   have_peak(false),
      pstack(nullptr), tstack(nullptr),
      psp(0), tsp(0), maxOpen(maxOpen)
{
    if (this->maxOpen < 2) this->maxOpen = 2; // minimal sensible capacity
    pstack  = new (std::nothrow) PeakState[this->maxOpen];
    tstack  = new (std::nothrow) TroughState[this->maxOpen];
}

PeakDetectorStage::~PeakDetectorStage() {
    delete last_data;
    delete[] pstack;
    delete[] tstack;
}

// MARK: --- PROCESSING ---

int PeakDetectorStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (!input || !input[0]) return -EINVAL;
    if (!pstack || !tstack)  return -ENOMEM;

    const struct sensor_data *in = input[0];
    float curr = decode_sample(in);

    // Bootstrap on very first call
    if (!this->last_data) {
        this->last_data = new sensor_data(*in);
        this->last_dx   = NAN;
        this->last_dir  = FLAT;
        // Update running mins/maxes for any open extremum (none yet)
        return 1;
    }

    // Compute derivative and direction
    float prev = decode_sample(this->last_data);
    float dx   = curr - prev;
    Dir dir    = deriv_dir(dx);
    Dir prev_dir = this->last_dir;

    // Swap in the latest full input for next round
    delete this->last_data;
    this->last_data = new sensor_data(*in);

    // Initialize last_dx on second sample
    if (isnan(this->last_dx)) {
        this->last_dx  = dx;
        this->last_dir = dir;
        // Update right-run stats for open peak/trough (still none normally)
        if (psp > 0) { float &rrm = pstack[psp-1].right_run_min; if (curr < rrm) rrm = curr; }
        if (tsp > 0) { float &rrM = tstack[tsp-1].right_run_max; if (curr > rrM) rrM = curr; }
        return 1;
    }

    /// MARK: --- TURN EVENTS ---

    // FALL -> RISE  => local minimum at 'prev'
    if (prev_dir == FALL && dir == RISE) {
        // This is a trough
        float left_base = have_peak ? last_peak : prev; // left base is last seen maximum
        TroughState v{ prev, left_base, -INFINITY };

        // Close any shallower troughs: new trough is strictly lower
        float cur_run_max = -INFINITY;
        while (tsp > 0 && v.val < tstack[tsp-1].val) {
            TroughState q = tstack[--tsp];
            float right_max = (q.right_run_max > cur_run_max) ? q.right_run_max : cur_run_max;
            float prom_left  = q.left_base_max - q.val;
            float prom_right = right_max        - q.val;
            float prominence = (prom_left < prom_right) ? prom_left : prom_right;

            // Emit finalized trough: peak_sign = -1
            *output = *in;
            const size_t prom_bytes = parseTypeSizes[this->parse_type];
            output->size = in->size + parseTypeSizes[PARSE_TYPE_INT8] + prom_bytes;

            // layout: [original bytes][int8_t peak_sign][prominence encoded as parse_type]
            std::memcpy(output->data, in->data, in->size);
            int8_t sign = -1;
            uint8_t* p_out = output->data + in->size;
            std::memcpy(p_out, &sign, sizeof(sign));
            encode_prominence(this->parse_type, prominence, p_out + sizeof(sign));

            // Cascade accumulation
            cur_run_max = (right_max > cur_run_max) ? right_max : cur_run_max;

            // Update state after emission and return immediately (one output per call)
            this->last_dx  = dx;
            this->last_dir = dir;
            if (psp > 0) { float &rrm = pstack[psp-1].right_run_min; if (curr < rrm) rrm = curr; }
            if (tsp > 0) { float &rrM = tstack[tsp-1].right_run_max; if (curr > rrM) rrM = curr; }
            // Record that we've seen a trough for left_base_min of the next peak
            have_valley = true;  last_valley = prev;
            have_peak   = false;
            return 0;
        }

        // Push the new trough (open)
        push_trough(v);

        // Bookkeeping: this valley serves as left base for the next max
        have_valley = true;  last_valley = prev;
        have_peak   = false;
    }

    // RISE -> FALL => local maximum at 'prev'
    if (prev_dir == RISE && dir == FALL) {
        // This is a peak
        float left_base = have_valley ? last_valley : prev; // left base is last seen valley
        PeakState p{ prev, left_base, INFINITY };

        // Close any smaller peaks: new peak is strictly higher
        float cur_run_min = INFINITY;
        while (psp > 0 && p.val > pstack[psp-1].val) {
            PeakState q = pstack[--psp];
            float right_min = (q.right_run_min < cur_run_min) ? q.right_run_min : cur_run_min;
            float prom_left  = q.val - q.left_base_min;
            float prom_right = q.val - right_min;
            float prominence = (prom_left < prom_right) ? prom_left : prom_right;

            // Emit finalized peak: peak_sign = +1
            *output = *in;
            const size_t prom_bytes = parseTypeSizes[this->parse_type];
            output->size = in->size + parseTypeSizes[PARSE_TYPE_INT8] + prom_bytes;

            // layout: [original bytes][int8_t peak_sign][prominence encoded as parse_type]
            std::memcpy(output->data, in->data, in->size);
            int8_t sign = +1;
            uint8_t* p_out = output->data + in->size;
            std::memcpy(p_out, &sign, sizeof(sign));
            encode_prominence(this->parse_type, prominence, p_out + sizeof(sign));

            // Cascade accumulation
            cur_run_min = (right_min < cur_run_min) ? right_min : cur_run_min;

            // Update state after emission and return immediately (one output per call)
            this->last_dx  = dx;
            this->last_dir = dir;
            if (psp > 0) { float &rrm = pstack[psp-1].right_run_min; if (curr < rrm) rrm = curr; }
            if (tsp > 0) { float &rrM = tstack[tsp-1].right_run_max; if (curr > rrM) rrM = curr; }
            // Record that we've seen a peak for left_base_max of the next trough
            have_peak   = true;  last_peak   = prev;
            have_valley = false;
            return 0;
        }

        // Push the new peak (open)
        push_peak(p);

        // Bookkeeping: this peak serves as left base for the next trough
        have_peak   = true;  last_peak   = prev;
        have_valley = false;
    }

    // --- PER-SAMPLE RUNNING UPDATES ---
    // Update right-run min/max for the most recent open extremum on each side
    if (psp > 0) {
        float &rrm = pstack[psp-1].right_run_min;
        if (curr < rrm) rrm = curr;
    }
    if (tsp > 0) {
        float &rrM = tstack[tsp-1].right_run_max;
        if (curr > rrM) rrM = curr;
    }

    // Keep derivative state
    this->last_dx  = dx;
    this->last_dir = dir;

    // No finalized extremum on this sample
    return 1;
}

float PeakDetectorStage::decode_sample(const struct sensor_data* sd) const {
    if (!sd) return 0.0f;
    const uint8_t* p = sd->data;
    return decode_as_float(this->parse_type, p);
}
