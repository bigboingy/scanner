/*
    Header file for icm20948
*/

// IMU Address
#define ICM20948_ADDRESS 0x69

// Magnetometer Address
#define AK09916_ADDRESS 0x0C

// Register bank selection
#define REG_BANK_SEL 0x7F

// User banks
// Stored as bank - address, where bank no. is shifted up 4 bits as required by REG_BANK_SEL
// User bank 0
#define WHO_AM_I             0x00 | 0 << 12
#define USER_CTRL            0x03 | 0 << 12
#define LP_CONFIG            0x05 | 0 << 12
#define PWR_MGMT_1           0x06 | 0 << 12
#define ACCEL_XOUT_H         0x2D | 0 << 12
#define EXT_SLV_SENS_DATA_00 0x3B | 0 << 12

// User bank 1
#define XA_OFFS_H   0x14 | 1 << 12
#define XA_OFFS_L   0x15 | 1 << 12
#define YA_OFFS_H   0x17 | 1 << 12
#define YA_OFFS_L   0x18 | 1 << 12
#define ZA_OFFS_H   0x1A | 1 << 12
#define ZA_OFFS_L   0x1B | 1 << 12


// User bank 2
#define GYRO_CONFIG_1 0x01 | 2 << 12

#define XG_OFFS_H     0x03 | 2 << 12
#define XG_OFFS_L     0x04 | 2 << 12
#define YG_OFFS_H     0x05 | 2 << 12
#define YG_OFFS_L     0x06 | 2 << 12
#define ZG_OFFS_H     0x07 | 2 << 12
#define ZG_OFFS_L     0x08 | 2 << 12

#define ODR_ALIGN_EN  0x09 | 2 << 12
#define ACCEL_CONFIG  0x14 | 2 << 12


// User bank 3, used for mag communication via i2c master
#define I2C_MST_CTRL  0x01 | 3 << 12
#define I2C_SLV0_ADDR 0x03 | 3 << 12
#define I2C_SLV0_REG  0x04 | 3 << 12
#define I2C_SLV0_CTRL 0x05 | 3 << 12
#define I2C_SLV0_DO   0x06 | 3 << 12

// Magnetometer registers
#define MAG_WHO_AM_I  0x01
#define MAG_HXL       0x11
#define MAG_CNTL2     0x31
#define MAG_CNTL3     0x32


