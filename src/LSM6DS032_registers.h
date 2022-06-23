//
// Created by robosam2003 on 16/06/2022.
//

#ifndef LSM6DS032TEST_LSM6DS032_REGISTERS_H
#define LSM6DS032TEST_LSM6DS032_REGISTERS_H

#define FUNC_CFG_ADDRESS            0x01    // R/W
#define PIN_CTRL                    0x02    // R/W
// reserved                         0x03-0x06
#define FIFO_CTRL1                  0x07    // R/W
#define FIFO_CTRL2                  0x08    // R/W
#define FIFO_CTRL3                  0x09    // R/W
#define FIFO_CTRL4                  0x0A    // R/W
#define COUNTER_BDR_REG1            0x0B    // R/W
#define COUNTER_BDR_REG2            0x0C    // R/W
#define INT1_CTRL                   0x0D    // R/W
#define INT2_CTRL                   0x0E    // R/W
#define WHO_AM_I                    0x0F    // R
#define CTRL1_XL                    0x10    // R/W
#define CTRL2_G                     0x11    // R/W
#define CTRL3_C                     0x12    // R/W
#define CTRL4_C                     0x13    // R/W
#define CTRL5_C                     0x14    // R/W
#define CTRL6_C                     0x15    // R/W
#define CTRL7_G                     0x16    // R/W
#define CTRL8_XL                    0x17    // R/W
#define CTRL9_XL                    0x18    // R/W
#define CTRL10_C                    0x19    // R/W
#define ALL_INT_SRC                 0x1A    // R
#define WAKE_UP_SRC                 0x1B    // R
#define TAP_SRC                     0x1C    // R
#define D6D_SRC                     0x1D    // R
#define STATUS_REG                  0x1E    // R
// reserved                         0x1F
#define OUT_TEMP_L                  0x20    // R
#define OUT_TEMP_H                  0x21    // R
#define OUTX_L_G                    0x22    // R
#define OUTX_H_G                    0x23    // R
#define OUTY_L_G                    0x24    // R
#define OUTY_H_G                    0x25    // R
#define OUTZ_L_G                    0x26    // R
#define OUTZ_H_G                    0x27    // R
#define OUTX_L_A                    0x28    // R
#define OUTX_H_A                    0x29    // R
#define OUTY_L_A                    0x2A    // R
#define OUTY_H_A                    0x2B    // R
#define OUTZ_L_A                    0x2C    // R
#define OUTZ_H_A                    0x2D    // R
// reserved                         0x2E-0x34
#define EMB_FUNC_STATUS_MAINPAGE    0x35    // R
#define FSM_STATUS_A_MAINPAGE       0x36    // R
#define FSM_STATUS_B_MAINPAGE       0x37    // R
// reserved                         0x38
#define STATUS_MASTER_MAINPAGE      0x39    // R
#define FIFO_STATUS1                0x3A    // R
#define FIFO_STATUS2                0x3B    // R
// reserved                         0x3C-0x3F
#define TIMESTAMP0                  0x40    // R
#define TIMESTAMP1                  0x41    // R
#define TIMESTAMP2                  0x42    // R
#define TIMESTAMP3                  0x43    // R
// reserved                         0x44-0x55
#define TAP_CFG0                    0x56    // R/W
#define TAP_CFG1                    0x57    // R/W
#define TAP_CFG2                    0x58    // R/W
#define TAP_THS_6D                  0x59    // R/W
#define INT_DUR2                    0x5A    // R/W
#define WAKE_UP_THS                 0x5B    // R/W
#define WAKE_UP_DUR                 0x5C    // R/W
#define FREE_FALL                   0x5D    // R/W
#define MD1_CFG                     0x5E    // R/W
#define MD2_CFG                     0x5F    // R/W
// reserved                         0x60-0x61
#define I3C_BUS_AVB                 0x62    // R/W
#define INTERNAL_FREQ_FINE          0x63    // R
// reserved                         0x64-0x72
#define X_OFS_USR                   0x73    // R/W
#define Y_OFS_USR                   0x74    // R/W
#define Z_OFS_USR                   0x75    // R/W
// reserved                         0x76-0x77
#define FIFO_DATA_OUT_TAG           0x78    // R
#define FIFO_DATA_OUT_X_L           0x79    // R
#define FIFO_DATA_OUT_X_H           0x7A    // R
#define FIFO_DATA_OUT_Y_L           0x7B    // R
#define FIFO_DATA_OUT_Y_H           0x7C    // R
#define FIFO_DATA_OUT_Z_L           0x7D    // R
#define FIFO_DATA_OUT_Z_H           0x7E    // R




#endif //LSM6DS032TEST_LSM6DS032_REGISTERS_H
