// Functions declarations
uint8_t imu_bank_sel(i2c_master_dev_handle_t imu_handle, uint8_t bank);
uint8_t imu_write( i2c_master_dev_handle_t imu_handle, uint8_t currentBank, uint16_t regAddress, uint8_t data );
uint8_t imu_read(i2c_master_dev_handle_t imu_handle, uint8_t currentBank, uint16_t regAddress, uint8_t *buffer, uint8_t noByes );
uint8_t imu_init(i2c_master_dev_handle_t imu_handle, uint8_t currentBank, uint8_t accScale, uint8_t gyroScale, uint16_t offsetNo);


