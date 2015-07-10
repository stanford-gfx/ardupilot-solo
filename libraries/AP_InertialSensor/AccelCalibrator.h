#include <AP_Math.h>

#define ACCEL_CAL_NUM_PARAMS 6
#define ACCEL_CAL_NUM_SAMPLES 300
#define MAX_ITERATIONS
enum accel_cal_status_t {
    ACCEL_CAL_NOT_STARTED=0,
    ACCEL_CAL_WAITING_FOR_ORIENTATION=1,
    ACCEL_CAL_COLLECTING_SAMPLE=2,
    ACCEL_CAL_SUCCESS=3,
    ACCEL_CAL_FAILED=4
};

enum accel_cal_fit_type_t {
    ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID=0,
    ACCEL_CAL_ELLIPSOID=1
};

class AccelCalibrator {
public:
    AccelCalibrator();

    void start(accel_cal_fit_type_t fit_type=ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, uint8_t num_samples=6, float sample_time=0.5f);
    void clear();

    void new_sample(const Vector3f& delta_velocity, float dt);

    void get_calibration(Vector3f& offsets, Vector3f& diagonals, Vector3f& offdiagonals);

    bool running() { return _status == ACCEL_CAL_REQUESTING_ORIENTATION || _status == ACCEL_CAL_COLLECTING_SAMPLE; }

    uint8_t get_sample_count() { return _samples_collected; }
    Vector3f get_sample(uint8_t num) { return _sample_buffer[num].delta_velocity/_sample_buffer[num].delta_time; }
private:
    struct accel_sample_t {
        Vector3f delta_velocity;
        float delta_time;
    };

    //configuration
    uint8_t _conf_num_samples;
    float _conf_sample_time;
    accel_cal_fit_t _conf_fit_type;

    // state
    accel_cal_status_t _status;
    accel_sample_t* _sample_buffer;
    uint8_t _samples_collected;
};
