#include <AP_Math.h>

enum accel_cal_status_t {
    ACCEL_CAL_NOT_STARTED=0,
    ACCEL_CAL_WAITING_FOR_ORIENTATION=1,
    ACCEL_CAL_COLLECTING_SAMPLE=2,
    ACCEL_CAL_SUCCESS=3,
    ACCEL_CAL_FAILED=4
};

class AccelCalibrator {
public:
    AccelCalibrator();

    void start(uint8_t num_samples=6, float sample_time=0.5f);
    void clear();

    void new_sample(const Vector3f& delta_velocity, float dt);

    void get_calibration(Vector3f& offsets, Vector3f& diagonals, Vector3f& offdiagonals);

    bool running() { return _status == ACCEL_CAL_REQUESTING_ORIENTATION || _status == ACCEL_CAL_COLLECTING_SAMPLE; }

    uint8_t get_sample_count() { return _samples_collected; }

private:
    struct accel_sample_t {
        Vector3f delta_velocity;
        float delta_time;
    };

    //configuration
    uint8_t _conf_num_samples;
    float _conf_sample_time;

    // state
    accel_cal_status_t _status;
    accel_sample_t* _sample_buffer;
    uint8_t _samples_collected;
};
