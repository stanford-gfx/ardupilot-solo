#ifndef DPS310_H_INCLUDED
#define DPS310_H_INCLUDED

/* Attributes: Product identification and version */

#define     VENDOR_NAME                                 "Infineon"
#define     DRIVER_NAME                                 "IFXDD"
#define     DEVICE_NAME                                 "Digital Barometric Pressure Sensor"
#define     DEVICE_MODEL_NAME                           "DPS310"
#define     DEVICE_HW_VERSION                           1.0
#define     DRIVER_VERSION                              1.0
#define     DEVICE_PROD_REV_ID                          0x00

/* Attributes: Device performance :Pressure Sensing */
#define     IFX_DPS310_PROD_REV_ID_REG_ADDR             0x0D
#define     IFX_DPS310_PROD_REV_ID_LEN                  1
#define     IFX_DSPS310_PROD_REV_ID_VAL                 DEVICE_PROD_REV_ID

#define     IFX_DPS310_SOFT_RESET_REG_ADDR              0x0C
#define     IFX_DPS310_SOFT_RESET_REG_DATA              0x09
#define     IFX_DPS310_SOFT_RESET_REG_LEN               1
#define     IFX_DPS310_SOFT_RESET_VERIFY_REG_ADDR       0x06

#define     IFX_DPS310_COEF_REG_ADDR                    0x10
#define     IFX_DPS310_COEF_LEN                         18    // Length in bytes

#define     IFX_DPS310_TMP_COEF_SRCE_REG_ADDR           0x28
#define     IFX_DPS310_TMP_COEF_SRCE_REG_LEN            1    // Length in bytes
#define     IFX_DPS310_TMP_COEF_SRCE_REG_POS_MASK       7    // Length in bytes

#define     IFX_DPS310_PSR_TMP_READ_REG_ADDR            0x00
#define     IFX_DPS310_PSR_TMP_READ_LEN                 6

#define     IFX_DPS310_PRS_CFG_REG_ADDR                 0x06
#define     IFX_DPS310_PRS_CFG_REG_LEN                  1

#define     IFX_DPS310_TMP_CFG_REG_ADDR                 0x07
#define     IFX_DPS310_TMP_CFG_REG_LEN                  1

#define     IFX_DPS310_MEAS_CFG_REG_ADDR                0x08
#define     IFX_DPS310_MEAS_CFG_REG_LEN                 1

#define     IFX_DPS310_CFG_REG_ADDR                     0x09
#define     IFX_DPS310_CFG_REG_LEN                      1

#define     IFX_DPS310_CFG_TMP_SHIFT_EN_SET_VAL         0x08
#define     IFX_DPS310_CFG_PRS_SHIFT_EN_SET_VAL         0x04


#define     IFX_DPS310_FIFO_READ_REG_ADDR               0x00
#define     IFX_DPS310_FIFO_REG_READ_LEN                3
#define     IFX_DPS310_FIFO_BYTES_PER_ENTRY             3

#define     IFX_DPS310_FIFO_FLUSH_REG_ADDR              0x0C
#define     IFX_DPS310_FIFO_FLUSH_REG_VAL               0b1000000U

#define     IFX_DPS310_CFG_SPI_MODE_POS                 0
#define     IFX_DPS310_CFG_SPI_MODE_3_WIRE_VAL          1
#define     IFX_DPS310_CFG_SPI_MODE_4_WIRE_VAL          0

#define     IFX_DPS310_CFG_FIFO_ENABLE_POS              1
#define     IFX_DPS310_CFG_FIFO_ENABLE_VAL              1
#define     IFX_DPS310_CFG_FIFO_DISABLE_VAL             0

#define     IFX_DPS310_CFG_INTR_PRS_ENABLE_POS          4
#define     IFX_DPS310_CFG_INTR_PRS_ENABLE_VAL          1U
#define     IFX_DPS310_CFG_INTR_PRS_DISABLE_VAL         0U

#define     IFX_DPS310_CFG_INTR_TEMP_ENABLE_POS         5
#define     IFX_DPS310_CFG_INTR_TEMP_ENABLE_VAL         1U
#define     IFX_DPS310_CFG_INTR_TEMP_DISABLE_VAL        0U

#define     IFX_DPS310_CFG_INTR_FIFO_FULL_ENABLE_POS    6
#define     IFX_DPS310_CFG_INTR_FIFO_FULL_ENABLE_VAL    1U
#define     IFX_DPS310_CFG_INTR_FIFO_FULL_DISABLE_VAL   0U

#define     IFX_DPS310_CFG_INTR_LEVEL_TYP_SEL_POS       7
#define     IFX_DPS310_CFG_INTR_LEVEL_TYP_ACTIVE_H      1U
#define     IFX_DPS310_CFG_INTR_LEVEL_TYP_ACTIVE_L      0U

#define     IFX_DPS310_INTR_SOURCE_PRESSURE             0
#define     IFX_DPS310_INTR_SOURCE_TEMPERATURE          1
#define     IFX_DPS310_INTR_SOURCE_BOTH                 2

#define     IFX_DPS310_INTR_STATUS_REG_ADDR             0x0A
#define     IFX_DPS310_INTR_STATUS_REG_LEN              1
#define     IFX_DPS310_INTR_DISABLE_ALL                (uint8_t)0b10001111

#define     EINVAL                                      1
#define     EIO                                         2

#ifndef NULL
#define     NULL                                        ((void*)0)
#endif // NULL


/* _______________________________________________________ */

#define POW_2_23_MINUS_1	0x7FFFFF   //implies 2^23-1
#define POW_2_24			0x1000000
#define POW_2_15_MINUS_1	0x7FFF
#define POW_2_16			0x10000
#define POW_2_11_MINUS_1	0x7FF
#define POW_2_12			0x1000
#define POW_2_20			0x100000
#define POW_2_19_MINUS_1	524287

/* _______________________________________________________ */

/*Some aliases*/
typedef unsigned char       uint8_t;

typedef char                s8;

typedef unsigned short      u16;

typedef short               int16_t;

typedef long                int32_t;

typedef	long long           s64;

typedef	unsigned long       u32;

typedef	unsigned long long  u64;

typedef float               f32;

typedef double              f64;

typedef uint8_t                  bool;

#define false               0
#define true                1

/* Struct to hold calibration coefficients read from device*/
typedef struct
{
  /* calibration registers */

  int16_t 	C0;	// 12bit
  int16_t 	C1;	// 12bit
  int32_t	C00;	// 20bit
  int32_t   C10;	// 20bit
  int16_t 	C01;	// 16bit
  int16_t	C11;	// 16bit
  int16_t	C20;	// 16bit
  int16_t	C21;	// 16bit
  int16_t	C30;	// 16bit

}dps310_cal_coeff_regs_s;


/* enum for seeting/getting device operating mode*/

typedef enum
{
  DPS310_MODE_IDLE                   =  0b00000000,
  DPS310_MODE_COMMAND_PRESSURE       =  0b00000001,
  DPS310_MODE_COMMAND_TEMPERATURE    =  0b00000010,
  DPS310_MODE_BACKGROUND_PRESSURE    =  0b00000101,
  DPS310_MODE_BACKGROUND_TEMPERATURE =  0b00000110,
  DPS310_MODE_BACKGROUND_ALL         =  0b00000111,

}dps310_operating_modes_e;



/* enum of scaling coefficients either Kp or Kt*/
typedef enum
{
    OSR_SF_1   = 524288,
    OSR_SF_2   = 1572864,
    OSR_SF_4   = 3670016,
    OSR_SF_8   = 7864320,
    OSR_SF_16  = 253952,
    OSR_SF_32  = 516096,
    OSR_SF_64  = 1040384,
    OSR_SF_128 = 2088960,

} dps310_scaling_coeffs_e;



/* enum of oversampling rates for pressure and temperature*/
typedef enum
{
    OSR_1   = 0b00000000,
    OSR_2   = 0b00000001,
    OSR_4   = 0b00000010,
    OSR_8   = 0b00000011,
    OSR_16  = 0b00000100,
    OSR_32  = 0b00000101,
    OSR_64  = 0b00000110,
    OSR_128 = 0b00000111,

} dps310_osr_e;



/* enum of measurement rates for pressure*/

typedef enum
{
    PM_MR_1   = 0b00000000,
    PM_MR_2   = 0b00010000,
    PM_MR_4   = 0b00100000,
    PM_MR_8   = 0b00110000,
    PM_MR_16  = 0b01000000,
    PM_MR_32  = 0b01010000,
    PM_MR_64  = 0b01100000,
    PM_MR_128 = 0b01110000,

} dps310_pm_rate_e;



/* enum of measurement rates for temperature*/

typedef enum
{
    TMP_MR_1   = 0b00000000,
    TMP_MR_2   = 0b00010000,
    TMP_MR_4   = 0b00100000,
    TMP_MR_8   = 0b00110000,
    TMP_MR_16  = 0b01000000,
    TMP_MR_32  = 0b01010000,
    TMP_MR_64  = 0b01100000,
    TMP_MR_128 = 0b01110000,

} dps310_tmp_rate_e;


/* enum of oversampling and measurement rates*/

typedef enum

{
    TMP_EXT_ASIC = 0x00,
    TMP_EXT_MEMS = 0x80,

}dps310_temperature_src_e;


class AP_Baro_DPS310 : public AP_Baro_Backend
{
public:
    // Constructor
    AP_Baro_DPS310(AP_Baro &baro);

    /* AP_Baro public interface: */
    void update();
    void accumulate(void);
private:
    dps310_scaling_coeffs_e   tmp_osr_scale_coeff;                    /* Temperature scaling coefficient*/
    dps310_scaling_coeffs_e   prs_osr_scale_coeff;                    /* Pressure scaling coefficient*/
    dps310_cal_coeff_regs_s   calib_coeffs;                           /* Calibration coefficients index */
    dps310_operating_modes_e  dev_mode;                               /* Current operating mode of device */
    dps310_pm_rate_e	      press_mr;				  /* Current measurement readout rate (ODR) for pressure */
    dps310_tmp_rate_e         temp_mr;				  /* Current measurement readout rate (ODR) for temperature */
    dps310_osr_e		      temp_osr;				  /* Current oversampling rate (OSR) for temperature */
    dps310_osr_e		      press_osr;				  /* Current oversampling rate (OSR) for pressure */
    dps310_temperature_src_e  tmp_ext;                                /* Temperature ASIC or MEMS. Should always be set MEMS*/
    uint8_t                   cfg_word;                               /* Keep the contents of CFG register as it gets configured
                                                                        to avoid excessive bus transactions */
    uint8_t                     _instance;
    float		                _temp_sum;
    float			            _press_sum;
    uint8_t			            _count;
    int32_t                     RawPress;
    int32_t                     RawTemp;

    int dps310_init ();
    int dps310_get_processed_data ();

    int dps310_config (dps310_osr_e osr_temp, dps310_tmp_rate_e mr_temp, dps310_osr_e osr_press, 
                       dps310_pm_rate_e mr_press, dps310_temperature_src_e temp_src);

    int dps310_standby();

    int dps310_resume();

};



#endif // DPS310_H_INCLUDED
