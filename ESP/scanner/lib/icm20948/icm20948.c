// Functions for communicating with icm20948
#include "icm20948_regs.h"
#include "driver/i2c_master.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"



// Function to select user bank for icm20948
// bank is 0,1,2,or 3, left shifted 4 bits
// returns bank number left shifted 4 bits
uint8_t imu_bank_sel(i2c_master_dev_handle_t imu_handle, uint8_t bank)
{
    // Send register then set bank
    uint8_t writeBuffer[2] = { REG_BANK_SEL, bank};
    ESP_ERROR_CHECK(i2c_master_transmit(imu_handle,writeBuffer,sizeof(writeBuffer),-1));
   
    // Return current bank
    return bank;
}

// Write function to icm20948
// Write order: IMU Adr -> Register Adr -> Data
// regAddress is 2 bytes: bank (high) and address (low)
// returns new bank number left shifted 4 bits
uint8_t imu_write( i2c_master_dev_handle_t imu_handle, uint8_t currentBank, uint16_t regAddress, uint8_t data )
{
    // Split regAddress into bank and address
    uint8_t bank = (regAddress>>8) & 0xFF; // Shift up one byte and mask with all 1s
    uint8_t address = regAddress & 0xFF; // Keep lower byte only
    uint8_t newBank = bank;
   
    // Are we at the right bank?
    if( bank != currentBank )
    {
        // Change bank
        newBank = imu_bank_sel(imu_handle,bank);
    }
   
    // Setup write buffer
    uint8_t writeBuffer[2] = {address, data};
   
    // Send command
    ESP_ERROR_CHECK(i2c_master_transmit(imu_handle,writeBuffer,sizeof(writeBuffer),-1));

    return newBank;
}

// Function to read a register from the icm20948
// regAddress is 2 bytes: bank (high) and address (low)
// buffer is a pointer to the buffer to write to
// noBytes is how many bytes of data to read. Register being read automatically increments at imu.
// returns new bank number left shifted 4 bits
uint8_t imu_read(i2c_master_dev_handle_t imu_handle, uint8_t currentBank, uint16_t regAddress, uint8_t *buffer, uint8_t noByes )
{
    // Split regAddress into bank and address
    uint8_t bank = (regAddress>>8) & 0xFF; // Shift up one byte and mask with all 1s
    uint8_t address = regAddress & 0xFF; // Keep lower byte only
    uint8_t newBank = bank;
   
    // Select user bank if required
    if( bank != currentBank )
    {
        // Change bank
        newBank = imu_bank_sel(imu_handle,bank);
    }

    // Read
    ESP_ERROR_CHECK(i2c_master_transmit_receive(imu_handle,&address,1,buffer,noByes,-1));

    return newBank;
}

// Function to write to magnetometer
uint8_t static mag_write(i2c_master_dev_handle_t imu_handle, uint8_t currentBank, uint16_t regAddress, uint8_t data)
{
    // Set slave 0 address on the chip's i2c master to the magnetometer's address
    currentBank = imu_write(imu_handle, currentBank, I2C_SLV0_ADDR, AK09916_ADDRESS);

    // Set the register we want to write to on the mag
    currentBank = imu_write(imu_handle, currentBank, I2C_SLV0_REG, regAddress);

    // Set data to write
    currentBank = imu_write(imu_handle, currentBank, I2C_SLV0_DO, data);

    // Send write command
    currentBank = imu_write(imu_handle, currentBank, I2C_SLV0_CTRL, 0x81); // Last 1 is important for sending 1 byte!!

    return currentBank;
}

// Function to get magnetometer to start filling external sensor registers
uint8_t static mag_read_setup(i2c_master_dev_handle_t imu_handle, uint8_t currentBank, uint8_t noBytes)
{
    // Set slave 0 address on the chip's i2c master to the magnetometer's address
    // Add a leading 1 bit to signify read
    currentBank = imu_write(imu_handle, currentBank, I2C_SLV0_ADDR, 0x80 | AK09916_ADDRESS);

    // Set the register we want to read from on the mag
    currentBank = imu_write(imu_handle, currentBank, I2C_SLV0_REG, MAG_HXL);

    // Send read command
    // Last 4 bits are set to how many bytes to read, reading 8 bytes
    // Magnetometer expects that the Control 2 register will be read
    currentBank = imu_write(imu_handle, currentBank, I2C_SLV0_CTRL, 0x80 | 8);

    vTaskDelay(20/portTICK_PERIOD_MS); // Delay

    return currentBank;
}

// Initialise icm20948
// Scales can be set to 0,1,2 or 3
// offsetNo is how many readings are averaged to set acc and gyro offset registers
// Returns current bank
uint8_t imu_init(i2c_master_dev_handle_t imu_handle, uint8_t currentBank, uint8_t accScale, uint8_t gyroScale, uint16_t offsetNo)
{
    // Reset bank to 0
    currentBank = imu_bank_sel(imu_handle,0<<4);

    // Reset chip
    currentBank = imu_write( imu_handle, currentBank, PWR_MGMT_1, 0xC1); // Set reset bit (autoclears), sleep bit and default clock bit
    vTaskDelay(100/portTICK_PERIOD_MS);

    // Turn on chip (clear sleep bit)
    currentBank = imu_write( imu_handle, currentBank, PWR_MGMT_1, 0x01);
    vTaskDelay(20/portTICK_PERIOD_MS);

    // Enable ODR start time alignment
    currentBank = imu_write( imu_handle, currentBank, ODR_ALIGN_EN, 0x01);
    vTaskDelay(20/portTICK_PERIOD_MS);

    // Set acc and gyro scales, maintains lowpass filters
    currentBank = imu_write(imu_handle, currentBank, ACCEL_CONFIG, (accScale << 1) | 0x01 );
    vTaskDelay(20/portTICK_PERIOD_MS);
    currentBank = imu_write(imu_handle, currentBank, GYRO_CONFIG_1, (gyroScale << 1) | 0x01 );
    vTaskDelay(20/portTICK_PERIOD_MS);

    // Reset and then enable I2C master
    currentBank = imu_write(imu_handle, currentBank, USER_CTRL, 0x02);
    vTaskDelay(100/portTICK_PERIOD_MS);
    currentBank = imu_write(imu_handle, currentBank, USER_CTRL, 0x20);
    vTaskDelay(20/portTICK_PERIOD_MS);

    // Set I2C master clock frequency to 400 kHz
    currentBank = imu_write(imu_handle, currentBank, I2C_MST_CTRL, 0x07);
    vTaskDelay(20/portTICK_PERIOD_MS);

    // Reset magnetometer and wait... If after offset setting, this resets za!!
    currentBank = mag_write(imu_handle, currentBank, MAG_CNTL3, 0x01);
    vTaskDelay(100/portTICK_PERIOD_MS);
    // Turn on mag and set to 100Hz... If after offset setting, this resets za!! (either one will, or in combination - mag write responsible?)
    currentBank = mag_write(imu_handle, currentBank, MAG_CNTL2, 0x08);
    vTaskDelay(100/portTICK_PERIOD_MS);

    // Do first read from magnetometer, after which it's read automatically at rate determined by gyro ODR, and stored in ext sensor regs
    currentBank = mag_read_setup(imu_handle,currentBank,8);

    // Setting offsets

    return currentBank;

}