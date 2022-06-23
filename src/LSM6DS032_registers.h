//
// Created by robosam2003 on 16/06/2022.
//

#ifndef LSM6DS032_REGISTERS_H
#define LSM6DS032_REGISTERS_H

// Dear GitHub copilot: You are truly appreciated. - Yours, Sam.

enum class LSM6DS032_REGISTER { // Registers are scoped to avoid confusion with other sensor registers names
    FUNC_CFG_ADDRESS           =  0x01,    // R/W
    PIN_CTRL                   =  0x02,
    // reserved                 0x03-0x06
    FIFO_CTRL1                 =  0x07,    // R/W
    FIFO_CTRL2                 =  0x08,    // R/W
    FIFO_CTRL3                 =  0x09,    // R/W
    FIFO_CTRL4                 =  0x0A,    // R/W
    COUNTER_BDR_REG1           =  0x0B,    // R/W
    COUNTER_BDR_REG2           =  0x0C,    // R/W
    INT1_CTRL                  =  0x0D,    // R/W
    INT2_CTRL                  =  0x0E,    // R/W
    WHO_AM_I                   =  0x0F,    // R
    CTRL1_XL                   =  0x10,    // R/W
    CTRL2_G                    =  0x11,    // R/W
    CTRL3_C                    =  0x12,    // R/W
    CTRL4_C                    =  0x13,    // R/W
    CTRL5_C                    =  0x14,    // R/W
    CTRL6_C                    =  0x15,    // R/W
    CTRL7_G                    =  0x16,    // R/W
    CTRL8_XL                   =  0x17,    // R/W
    CTRL9_XL                   =  0x18,    // R/W
    CTRL10_C                   =  0x19,    // R/W
    ALL_INT_SRC                =  0x1A,    // R
    WAKE_UP_SRC                =  0x1B,    // R
    TAP_SRC                    =  0x1C,    // R
    D6D_SRC                    =  0x1D,    // R
    STATUS_REG                 =  0x1E,
    // reserved                   0x1F
    OUT_TEMP_L                 =  0x20,    // R
    OUT_TEMP_H                 =  0x21,    // R
    OUTX_L_G                   =  0x22,    // R
    OUTX_H_G                   =  0x23,    // R
    OUTY_L_G                   =  0x24,    // R
    OUTY_H_G                   =  0x25,    // R
    OUTZ_L_G                   =  0x26,    // R
    OUTZ_H_G                   =  0x27,    // R
    OUTX_L_A                   =  0x28,    // R
    OUTX_H_A                   =  0x29,    // R
    OUTY_L_A                   =  0x2A,    // R
    OUTY_H_A                   =  0x2B,    // R
    OUTZ_L_A                   =  0x2C,    // R
    OUTZ_H_A                   =  0x2D,
    // reserved                 0x2E-0x34
    EMB_FUNC_STATUS_MAINPAGE   =  0x35,    // R
    FSM_STATUS_A_MAINPAGE      =  0x36,    // R
    FSM_STATUS_B_MAINPAGE      =  0x37,
    // reserved                   0x38
    STATUS_MASTER_MAINPAGE     =  0x39,    // R
    FIFO_STATUS1               =  0x3A,    // R
    FIFO_STATUS2               =  0x3B,
    // reserved                 0x3C-0x3F
    TIMESTAMP0                 =  0x40,    // R
    TIMESTAMP1                 =  0x41,    // R
    TIMESTAMP2                 =  0x42,    // R
    TIMESTAMP3                 =  0x43,
    // reserved                 0x44-0x55
    TAP_CFG0                   =  0x56,    // R/W
    TAP_CFG1                   =  0x57,    // R/W
    TAP_CFG2                   =  0x58,    // R/W
    TAP_THS_6D                 =  0x59,    // R/W
    INT_DUR2                   =  0x5A,    // R/W
    WAKE_UP_THS                =  0x5B,    // R/W
    WAKE_UP_DUR                =  0x5C,    // R/W
    FREE_FALL                  =  0x5D,    // R/W
    MD1_CFG                    =  0x5E,    // R/W
    MD2_CFG                    =  0x5F,
    // reserved                 0x60-0x61
    I3C_BUS_AVB                =  0x62,    // R/W
    INTERNAL_FREQ_FINE         =  0x63,
    // reserved                 0x64-0x72
    X_OFS_USR                  =  0x73,    // R/W
    Y_OFS_USR                  =  0x74,    // R/W
    Z_OFS_USR                  =  0x75,
    // reserved                 0x76-0x77
    FIFO_DATA_OUT_TAG          =  0x78,    // R
    FIFO_DATA_OUT_X_L          =  0x79,    // R
    FIFO_DATA_OUT_X_H          =  0x7A,    // R
    FIFO_DATA_OUT_Y_L          =  0x7B,    // R
    FIFO_DATA_OUT_Y_H          =  0x7C,    // R
    FIFO_DATA_OUT_Z_L          =  0x7D,    // R
    FIFO_DATA_OUT_Z_H          =  0x7E     // R
};



#endif //LSM6DS032_REGISTERS_H
