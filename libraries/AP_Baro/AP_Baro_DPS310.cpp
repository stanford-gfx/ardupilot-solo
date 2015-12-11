/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  AP_Baro_DPS310 barometer driver.
    (C) Siddharth B Purohit, 3DRobotics Inc.
*/

#include <AP_HAL.h>
#include <AP_Common.h>

#include "AP_Baro.h"

extern const AP_HAL::HAL& hal;
AP_Baro_DPS310::AP_Baro_DPS310(AP_Baro &baro) :
    AP_Baro_Backend(baro),
    _instance(0),
    _temp_sum(0),
    _press_sum(0),
    _count(0)
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("DPS310: unable to get semaphore"));
    }
    uint8_t state;
    cfg_word = 0;
    enable = 0;
   
    /*first verify chip by reading product and rev id*/
    hal.i2c->readRegister(BMP085_ADDRESS, IFX_DPS310_PROD_REV_ID_REG_ADDR, state);

    if (state == 0){
        hal.scheduler->panic(PSTR("DPS310: Failed to Read Product and Rev Id"));
    }

    if (state != IFX_DSPS310_PROD_REV_ID_VAL){
        hal.scheduler->panic(PSTR("DPS310: Invalid Read Product and Rev Id"));
    }

    /* read now the calibration coeffs, temperature coef source and store in driver state*/
    state = dps310_read_calib_coeffs();

    if (state == 0){
        hal.scheduler->panic(PSTR("DPS310: Failled to read Calibration Coefs"));
    }

    /* configure sensor for default ODR settings*/
    state = dps310_config(IFX_DPS310_TEMPERATURE_OSR,
                        IFX_DPS310_TEMPERATURE_MR,
                        IFX_DPS310_PRESSURE_OSR,
                        IFX_DPS310_PRESSURE_MR,
                        tmp_ext);
    if (state == 0){
        hal.scheduler->panic(PSTR("DPS310: Failed to Configure"));
    }

    /* activate sensor*/
    state = dps310_resume(drv_state);
    if (state == 0){
        hal.scheduler->panic(PSTR("DPS310: Baro Init Failed!!"));
    }
    
    _instance = _frontend.register_sensor();
}


uint8_t read_calib_coeffs()
{
        uint8_t ret, reg_temp_ext;
        uint8_t read_buffer[IFX_DPS310_COEF_LEN] = {0};

        ret =  hal.i2c->readRegisters(DPS310_ADDRESS, IFX_DPS310_COEF_REG_ADDR, IFX_DPS310_COEF_LEN, read_buffer);

        if ( ret == 0 ) {
            return 0;
        }

        calib_coeffs.C0 = (read_buffer[0] << 4) + ((read_buffer[1] >>4) & 0x0F);

        if(calib_coeffs.C0 > POW_2_11_MINUS_1)
            calib_coeffs.C0 = calib_coeffs.C0 - POW_2_12;

        calib_coeffs.C1 = (read_buffer[2] + ((read_buffer[1] & 0x0F)<<8));

        if(calib_coeffs.C1 > POW_2_11_MINUS_1)
            calib_coeffs.C1 = calib_coeffs.C1 - POW_2_12;

        calib_coeffs.C00 = ((read_buffer[4]<<4) + (read_buffer[3]<<12)) + ((read_buffer[5]>>4) & 0x0F);

        if(calib_coeffs.C00 > POW_2_19_MINUS_1)
            calib_coeffs.C00 = calib_coeffs.C00 -POW_2_20;

        calib_coeffs.C10 = ((read_buffer[5] & 0x0F)<<16) + read_buffer[7] + (read_buffer[6]<<8);

        if(calib_coeffs.C10 > POW_2_19_MINUS_1)
            calib_coeffs.C10 = calib_coeffs.C10 - POW_2_20;

        calib_coeffs.C01 = (read_buffer[9] + (read_buffer[8]<<8));

        if(calib_coeffs.C01 > POW_2_15_MINUS_1)
            calib_coeffs.C01 = calib_coeffs.C01 - POW_2_16;

        calib_coeffs.C11 = (read_buffer[11] + (read_buffer[10]<<8));

        if(calib_coeffs.C11 > POW_2_15_MINUS_1)
            calib_coeffs.C11 = calib_coeffs.C11 - POW_2_16;

        calib_coeffs.C20 = (read_buffer[13] + (read_buffer[12]<<8));

        if(calib_coeffs.C20 > POW_2_15_MINUS_1)
            calib_coeffs.C20 = calib_coeffs.C20 - POW_2_16;

        calib_coeffs.C21 = (read_buffer[15] + (read_buffer[14]<<8));

        if(calib_coeffs.C21 > POW_2_15_MINUS_1)
            calib_coeffs.C21 = calib_coeffs.C21 - POW_2_16;

        calib_coeffs.C30 = (read_buffer[17] + (read_buffer[16]<<8));

        if(calib_coeffs.C30 > POW_2_15_MINUS_1)
            calib_coeffs.C30 = calib_coeffs.C30 - POW_2_16;

        /* lets see which temperature diode is used for calibration and update state accordingly*/
        hal.i2c->readRegister(DPS310_ADDRESS,IFX_DPS310_TMP_COEF_SRCE_REG_ADDR,reg_temp_ext);

        if ((reg_temp_ext >> IFX_DPS310_TMP_COEF_SRCE_REG_POS_MASK) & 1 ){
				tmp_ext = TMP_EXT_MEMS;
		} else{
				tmp_ext = TMP_EXT_ASIC;
		}
        return 1;
}
