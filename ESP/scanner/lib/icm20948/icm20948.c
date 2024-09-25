// Functions for communicating with icm20948
#include "icm20948_regs.h"
#include "driver/i2c_master.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h" // Need for delays
#include "esp_log.h"

// Logging tag
static const char *TAG = "IMU";

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

    // Add a delay so the sensor can work itself out, won't affect normal operation as no writes are made
    vTaskDelay(50/portTICK_PERIOD_MS);

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

    // What scale factors correspond to accScale and gyroScale settings?
    float accScaleFacs[4] = {16384,8192,4096,2048}; float accScaleFac = accScaleFacs[accScale]; // LSB/g
    float gyroScaleFacs[4] = {131,65.5,32.8,16.4}; float gyroScaleFac = gyroScaleFacs[gyroScale]; //LSB/dps

    // Reset bank to 0
    currentBank = imu_bank_sel(imu_handle,0<<4);

    // Reset chip
    currentBank = imu_write( imu_handle, currentBank, PWR_MGMT_1, 0xC1); // Set reset bit (autoclears), sleep bit and default clock bit
    vTaskDelay(100/portTICK_PERIOD_MS);

    // Turn on chip (clear sleep bit)
    currentBank = imu_write( imu_handle, currentBank, PWR_MGMT_1, 0x01);

    // Enable ODR start time alignment
    currentBank = imu_write( imu_handle, currentBank, ODR_ALIGN_EN, 0x01);

    // Set acc and gyro scales, maintains lowpass filters
    currentBank = imu_write(imu_handle, currentBank, ACCEL_CONFIG, (accScale << 1) | 0x01 );
    currentBank = imu_write(imu_handle, currentBank, GYRO_CONFIG_1, (gyroScale << 1) | 0x01 );
    ESP_LOGI(TAG,"Initialising IMU with acc scale %i and gyro scale %i",accScale,gyroScale);

    // Reset and then enable I2C master
    currentBank = imu_write(imu_handle, currentBank, USER_CTRL, 0x02);
    vTaskDelay(100/portTICK_PERIOD_MS);
    currentBank = imu_write(imu_handle, currentBank, USER_CTRL, 0x20);

    // Set I2C master clock frequency to 400 kHz
    currentBank = imu_write(imu_handle, currentBank, I2C_MST_CTRL, 0x07);

    // Reset magnetometer and wait... If after offset setting, this resets za!!
    currentBank = mag_write(imu_handle, currentBank, MAG_CNTL3, 0x01);
    vTaskDelay(100/portTICK_PERIOD_MS);
    // Turn on mag and set to 100Hz... If after offset setting, this resets za!! (either one will, or in combination - mag write responsible?)
    currentBank = mag_write(imu_handle, currentBank, MAG_CNTL2, 0x08);
    vTaskDelay(100/portTICK_PERIOD_MS);

    // Do first read from magnetometer, after which it's read automatically at rate determined by gyro ODR, and stored in ext sensor regs
    currentBank = mag_read_setup(imu_handle,currentBank,8);

    // Setting offsets
    uint8_t temp[12]; // Will need to cast to signed types
    int32_t accSum[3] = {0,0,0}; // x,y,z, signed!
    int32_t gyroSum[3] = {0,0,0};
    for( uint16_t i=0; i<offsetNo; i++){ // Take offsetNo reads
        currentBank = imu_read(imu_handle, currentBank, ACCEL_XOUT_H,temp,12); // Read
        accSum[0] += (int16_t)(temp[0] << 8 | temp[1]); // Add to summation. Needs to be int16 to capture -ve vals!
        accSum[1] += (int16_t)(temp[2] << 8 | temp[3]);
        accSum[2] += (int16_t)(temp[4] << 8 | temp[5]);
        gyroSum[0] += (int16_t)(temp[6] << 8 | temp[7]);
        gyroSum[1] += (int16_t)(temp[8] << 8 | temp[9]);
        gyroSum[2] += (int16_t)(temp[10] << 8 | temp[11]);
    }
    // Acc offset registers take a 15 bit value. Need to read seperately as regs aren't sequential!
    currentBank = imu_read(imu_handle,currentBank,XA_OFFS_H,&temp[0],2); // Get current offsets (15bit), put into temp[0:6]
    currentBank = imu_read(imu_handle,currentBank,YA_OFFS_H,&temp[2],2);
    currentBank = imu_read(imu_handle,currentBank,ZA_OFFS_H,&temp[4],2);
    // Averages must be converted into +/-32 G scale, which isn't even a setting, equates to 1024 lsb/G or 0.98mg steps
    uint16_t accOffsScaleFac = 1024;
    int16_t accAv[3] = {accSum[0]*(accOffsScaleFac/accScaleFac)/offsetNo + 0.5, // Add 0.5 for rounding
                        accSum[1]*(accOffsScaleFac/accScaleFac)/offsetNo + 0.5,
                        accSum[2]*(accOffsScaleFac/accScaleFac)/offsetNo + 0.5}; // Converted averages in 1024 lsb/G
    int16_t accOffsets[3] = { // Work out the offsets
        (temp[0] << 7 | temp[1] >> 1) - accAv[0], // Current - error
        (temp[2] << 7 | temp[3] >> 1) - accAv[1], // xxxxxxxx xxxxxxxo --> oxxxxxxx xxxxxxxx
        (temp[4] << 7 | temp[5] >> 1) - (accAv[2]-(1*accOffsScaleFac)) // Starting error for z is distance from 1
    }; // oxxxxxxx xxxxxxx --> xxxxxxxx xxxxxxo
    currentBank = imu_write(imu_handle,currentBank,XA_OFFS_H,(uint8_t)(accOffsets[0]>>7)); // Acc x high
    currentBank = imu_write(imu_handle,currentBank,XA_OFFS_L,(uint8_t)(accOffsets[0]<<1)); // Acc x low
    currentBank = imu_write(imu_handle,currentBank,YA_OFFS_H,(uint8_t)(accOffsets[1]>>7)); // Acc y high
    currentBank = imu_write(imu_handle,currentBank,YA_OFFS_L,(uint8_t)(accOffsets[1]<<1)); // Acc y low
    currentBank = imu_write(imu_handle,currentBank,ZA_OFFS_H,(uint8_t)(accOffsets[2]>>7)); // Acc z high
    currentBank = imu_write(imu_handle,currentBank,ZA_OFFS_L,(uint8_t)(accOffsets[2]<<1)); // Acc z low
    ESP_LOGI(TAG,"Adjusted acceleration offsets by x: %"PRId16" y: %"PRId16" z: %"PRId16" (1024 lsb/G)",
            -accAv[0],-accAv[1],-(accAv[2]-(1*accOffsScaleFac)));

    // Gyro
    // Average must be converted into +/-1000 dps scale, (setting 2)
    int16_t gyroAv[3] = {gyroSum[0]*(gyroScaleFacs[2]/gyroScaleFac)/offsetNo + 0.5, // Add 0.5 for rounding
                         gyroSum[1]*(gyroScaleFacs[2]/gyroScaleFac)/offsetNo + 0.5,
                         gyroSum[2]*(gyroScaleFacs[2]/gyroScaleFac)/offsetNo + 0.5}; // Converted averages

    currentBank = imu_write(imu_handle,currentBank,XG_OFFS_H,(uint8_t)(-gyroAv[0]>>8)); // Gyro x high
    currentBank = imu_write(imu_handle,currentBank,XG_OFFS_L,(uint8_t)(-gyroAv[0]&0xFF)); // Gyro x low
    currentBank = imu_write(imu_handle,currentBank,YG_OFFS_H,(uint8_t)(-gyroAv[1]>>8)); // Gyro y high
    currentBank = imu_write(imu_handle,currentBank,YG_OFFS_L,(uint8_t)(-gyroAv[1]&0xFF)); // Gyro y low
    currentBank = imu_write(imu_handle,currentBank,ZG_OFFS_H,(uint8_t)(-gyroAv[2]>>8)); // Gyro z high
    currentBank = imu_write(imu_handle,currentBank,ZG_OFFS_L,(uint8_t)(-gyroAv[2]&0xFF)); // Gyro z low
    ESP_LOGI(TAG,"Set gyroscope offsets in x: %"PRId16" y: %"PRId16" z: %"PRId16" (32.8 LSB/dps)",
            gyroAv[0],gyroAv[1],gyroAv[2]);

    return currentBank;

}