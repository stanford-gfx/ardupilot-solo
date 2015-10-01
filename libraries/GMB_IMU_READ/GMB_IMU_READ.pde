// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Test for AP_GPS_AUTO
//

// #include <stdlib.h>

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <GCS_MAVLink.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Math.h>
#include <AP_Notify.h>
#include <AP_BoardLED.h>
#include <AP_NavEKF.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>
#include <AP_BattMonitor.h>
#include <AP_SerialManager.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// create board led object
AP_BoardLED board_led;

AP_SerialManager serial_manager;

void setup()
{
    // initialise the leds
    board_led.init();

    /* Allocate large enough buffers on uartA to support mavlink */
    hal.uartA->begin(115200, 128, 256);
    hal.console->backend_open();
}

void loop()
{
    int a = hal.console->available();
    if (a > 0) {
        hal.console->print("Console loopback:");
        int r = hal.console->read();
        while (r > 0) {
            hal.console->write( (uint8_t) r );
            r = hal.console->read();
        }
        hal.console->println();
    }
}

AP_HAL_MAIN();
