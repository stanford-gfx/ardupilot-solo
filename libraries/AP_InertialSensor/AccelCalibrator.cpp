/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AccelCalibrator.h"

AccelCalibrator::AccelCalibrator() {
    clear();
}

void AccelCalibrator::start(uint8_t num_samples, float sample_time) {
    if(running(_status != ACCEL_CAL_NOT_STARTED)) {
        return;
    }

    _conf_num_samples = num_samples;
    _conf_sample_time = sample_time;

    set_status(ACCEL_CAL_WAITING_FOR_ORIENTATION);
}

void AccelCalibrator::orient_done() {
    set_status(ACCEL_CAL_COLLECTING_SAMPLE);
}

void AccelCalibrator::new_sample(const Vector3f& delta_velocity, float dt) {
    if (_status != ACCEL_CAL_COLLECTING_SAMPLE) {
        return;
    }

    _sample_buffer[_samples_collected].delta_velocity += delta_velocity;
    _sample_buffer[_samples_collected].delta_time += dt;

    if (_sample_buffer[_samples_collected].delta_time > _conf_sample_time) {
        _samples_collected++;

        if (_samples_collected >= _conf_num_samples) {
            // TODO TODO TODO TODO TODO TODO TODO TODO
            // - determine type of fit to run based on number of samples
            // - fail or fallback if sample distribution is bad for fit type
            // - run fit
            // - call set_status(ACCEL_CAL_SUCCESS) or set_status(ACCEL_CAL_FAILED)
            // TODO TODO TODO TODO TODO TODO TODO TODO
        }
    }
}

void AccelCalibrator::clear() {
    set_status(ACCEL_CAL_NOT_STARTED);
}

void AccelCalibrator::reset_state() {
    _samples_collected = 0;
}

bool AccelCalibrator::set_status(accel_cal_status_t status) {
    switch (status) {
        case ACCEL_CAL_NOT_STARTED:
            reset_state();
            _status = ACCEL_CAL_NOT_STARTED;

            if (_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }

            return true;

        case ACCEL_CAL_WAITING_FOR_ORIENTATION:
            if (_samples_collected >= _num_samples) {
                return false;
            }
            _status = ACCEL_CAL_WAITING_FOR_ORIENTATION
            return true;

        case ACCEL_CAL_COLLECTING_SAMPLE:
            if (_status != ACCEL_CAL_WAITING_FOR_ORIENTATION) {
                return false;
            }
            _status = ACCEL_CAL_COLLECTING_SAMPLE;
            return true;

        case ACCEL_CAL_SUCCESS:
            if (_status != ACCEL_CAL_COLLECTING_SAMPLE) {
                return false;
            }

            if (_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }

            _status = ACCEL_CAL_SUCCESS;
            return true;

        case ACCEL_CAL_FAILED:
            if (_status == COMPASS_CAL_NOT_STARTED) {
                return false;
            }

            if (_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }

            _status = ACCEL_CAL_FAILED;
            return true;
    };
}
