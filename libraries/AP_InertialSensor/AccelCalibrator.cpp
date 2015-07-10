/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AccelCalibrator.h"

AccelCalibrator::AccelCalibrator() {
    clear();
}

void AccelCalibrator::start(accel_cal_fit_t fit_type, uint8_t num_samples, float sample_time) {
    if(running()) {
        return;
    }

    _conf_num_samples = num_samples;
    _conf_sample_time = sample_time;
    _conf_fit_type = fit_type;

    switch (_conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
            if (_conf_num_samples < 6) {
                set_status(ACCEL_CAL_FAILED);
                return;
            }
            break;
        case ACCEL_CAL_ELLIPSOID:
            if (_conf_num_samples < 8) {
                set_status(ACCEL_CAL_FAILED);
                return;
            }
            break;
    }

    set_status(ACCEL_CAL_WAITING_FOR_ORIENTATION);
}

void AccelCalibrator::orient_done() {
    set_status(ACCEL_CAL_COLLECTING_SAMPLE);
}
bool AccelCalibrator::is_sample_valid(const Vector3f& sample)
{
    uint8_t faces = 2*_conf_num_samples-4;
    float theta = acosf(cosf((4.0f*PI/(3.0f*faces)) + PI/3.0f)/(1.0f-cosf((4.0f*PI/(3.0f*faces)) + PI/3.0f)));
    float min_distance = GRAVITY_MSS * 2*sinf(theta/2);
    
    min_distance *= 0.5f;

    for(uint8_t cnt; cnt < _conf_num_samples; cnt++) {
        if((get_sample(cnt) - sample).length() < min_distance){
            return false;
        }
    }
    return true;
}
void AccelCalibrator::new_sample(const Vector3f& delta_velocity, float dt) {
    if (_status != ACCEL_CAL_COLLECTING_SAMPLE) {
        return;
    }

    _sample_buffer[_samples_collected].delta_velocity += delta_velocity;
    _sample_buffer[_samples_collected].delta_time += dt;

    if (_sample_buffer[_samples_collected].delta_time > _conf_sample_time) {
        if(!is_sample_valid(get_sample(_samples_collected))) {
            set_status(ACCEL_CAL_FAILED);
            return;
        }
        _samples_collected++;

        if (_samples_collected >= _conf_num_samples) {
            run_fit(MAX_ITERATIONS);
            set_status(ACCEL_CAL_SUCCESS);
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
            if (_status == ACCEL_CAL_NOT_STARTED) {
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

/////////////////////////////////////////////////////////////
////////////////////// PRIVATE METHODS //////////////////////
/////////////////////////////////////////////////////////////

float AccelCalibrator::calc_residual(const Vector3f& sample, const param_t& params) const {
    Matrix3f scale_factor(
        params.diag.x    , 0                , 0,
        0                , params.diag.y    , 0,
        0                , 0                , params.diag.z
    );
    return GRAVITY_MSS - (scale_factor*(sample+params.offset)).length();
}

float AccelCalibrator::calc_mean_squared_residuals() const
{
    return calc_mean_squared_residuals(_params);
}

float AccelCalibrator::calc_mean_squared_residuals(const param_t& params) const
{
    if(_sample_buffer == NULL || _samples_collected == 0) {
        return 1.0e30f;
    }
    float sum = 0.0f;
    for(uint16_t i=0; i < _samples_collected; i++){
        Vector3f sample = get_sample(i);
        float resid = calc_residual(sample, params);
        sum += sq(resid);
    }
    sum /= _samples_collected;
    return sum;
}

void AccelCalibrator::calc_jacob(const Vector3f& sample, const param_t& params, float* ret) const{

    Matrix3f scale_factor(
        params.diag.x    , 0                , 0,
        0                , params.diag.y    , 0,
        0                , 0                , params.diag.z
    );
    float length = (scale_factor*(sample+params.offset)).length();

    // 0-2: scale_factors
    ret[0] = -1.0f * (params.diag.x * ((params.diag.x * sample.x) - params.offset.x)) / length;
    ret[1] = -1.0f * (params.diag.y * ((params.diag.y * sample.y) - params.offset.y)) / length;
    ret[2] = -1.0f * (params.diag.z * ((params.diag.z * sample.z) - params.offset.z)) / length;

    // 3-5: offset
    ret[3] = ((params.diag.x * sample.x) - params.offset.x)/length;
    ret[4] = ((params.diag.y * sample.y) - params.offset.y)/length;
    ret[5] = ((params.diag.z * sample.z) - params.offset.z)/length;
}

void AccelCalibrator::run_fit(uint8_t max_iterations)
{
    if(_sample_buffer == NULL) {
        return;
    }
    float fitness = _fitness;
    param_t fit_param = _param;

    for(uint8_t iterations=0; iterations<max_iterations || _fitness > 0.000000001f; iterations++) {
        float JTJ[ACCEL_CAL_NUM_PARAMS*ACCEL_CAL_NUM_PARAMS];
        float JTFI[ACCEL_CAL_NUM_PARAMS];

        memset(&JTJ,0,sizeof(JTJ));
        memset(&JTFI,0,sizeof(JTFI));

        for(uint16_t k = 0; k<_samples_collected; k++) {
            Vector3f sample = get_sample(k);

            param_t sphere_jacob;

            calc_jacob(sample, fit_param, sphere_jacob);

            for(uint8_t i = 0;i < ACCEL_CAL_NUM_PARAMS; i++) {
                // compute JTJ
                for(uint8_t j = 0; j < ACCEL_CAL_NUM_PARAMS; j++) {
                    JTJ[i*ACCEL_CAL_NUM_PARAMS+j] += sphere_jacob.array[i] * sphere_jacob.array[j];
                }
                // compute JTFI
                JTFI[i] += sphere_jacob.array[i] * calc_residual(sample, fit_param, _ellipsoid_param);
            }
        }

        if(!inverse4x4(JTJ, JTJ)) {
            return;
        }

        for(uint8_t row=0; row < ACCEL_CAL_NUM_PARAMS; row++) {
            for(uint8_t col=0; col < ACCEL_CAL_NUM_PARAMS; col++) {
                fit_param.array[row] -= JTFI[col] * JTJ[row*ACCEL_CAL_NUM_PARAMS+col];
            }
        }

        fitness = calc_mean_squared_residuals(fit_param,_ellipsoid_param);

        if(fitness < _fitness) {
            _fitness = fitness;
            _sphere_param = fit_param;
        }
    }
}
