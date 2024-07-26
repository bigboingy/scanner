/*
main.c
tracked in git as mainlink.c
*/
#include "project.h"
#include "math.h"
#include "icm20948.h"

// GLOBALS
// Variable to keep track of current icm20948 register bank (inits to 0)
// Note that banks are stored in the form they are sent: 0,1,2,3 shifted up 4 bits
uint8 currentBank; // Set initially by imu_init call. Only changed by sel_imu_bank

// mode functions
#define LIDAR_ON       1<<0
#define IMU_ON         1<<1
#define CONT_MODE       1<<2 // If this bit is set, keep sending data even when count=0
#define COUNT_SHIFT     3    // How many bits forward do you have to shift mode to get the count?

// FUNCTIONS
// Function to select user bank for icm20948
// Register address is always the same
// bank is 0,1,2,or 3, shifted up 4 bits
void imu_sel_bank(uint8 bank)
{
    // Prepare data to write: bank select register followed by desired bank
    uint8 writeBuffer[2] = { REG_BANK_SEL, bank};
    I2C_MasterWriteBuf(ICM20948_ADDRESS, (uint8*)writeBuffer, sizeof(writeBuffer), I2C_MODE_COMPLETE_XFER);

    // Wait for completion
    while(I2C_MasterStatus() & I2C_MSTAT_XFER_INP);
   
    // Update current bank tracker
    currentBank = bank;
}

// Write function to icm20948
// Write order: Adr -> Register Adr -> Data
// regAddress is 2 bytes: bank (high) and address (low)
// data is a single byte of data to send
void imu_write( uint16 regAddress, uint8 data)
{
    // Split regAddress into bank and address
    uint8 bank = (regAddress>>8) & 0xFF; // Shift up one byte and mask with all 1s
    uint8 address = regAddress & 0xFF; // Keep lower byte only
   
    // Are we at the right bank?
    if( bank != currentBank )
    {
        // Change bank
        imu_sel_bank(bank);
    }
   
    // Setup write buffer
    uint8 writeBuffer[2] = {address, data};
   
    // Send command
    I2C_MasterWriteBuf(ICM20948_ADDRESS, (uint8*)writeBuffer, sizeof(writeBuffer), I2C_MODE_COMPLETE_XFER);
   
    // Wait
    while(I2C_MasterStatus() & I2C_MSTAT_XFER_INP);
}

// Function to read a register from the icm20948
// regAddress is 2 bytes: bank (high) and address (low)
// bufferPointer is a pointer to the buffer to write to
// numBytes is how many bytes of data to read. Register being read automatically increments at imu.
void imu_read( uint16 regAddress, uint8*bufferPointer, uint8 numByes)
{
    // Split regAddress into bank and address
    uint8 bank = (regAddress>>8) & 0xFF; // Shift up one byte and mask with all 1s
    uint8 address = regAddress & 0xFF; // Keep lower byte only
   
    // Select user bank if required
    if( bank != currentBank )
    {
        // Change bank
        imu_sel_bank(bank);
    }
   
    // Write operation (sending register address)
    I2C_MasterWriteBuf(ICM20948_ADDRESS, &address, 1, I2C_MODE_COMPLETE_XFER);
   
    // Wait for completion
    while(I2C_MasterStatus() & I2C_MSTAT_XFER_INP); // Continues when false, all zeros
   
    // Read operation
    I2C_MasterReadBuf(ICM20948_ADDRESS, bufferPointer, numByes, I2C_MODE_COMPLETE_XFER);
   
    // Wait for completion
    while(I2C_MasterStatus() & I2C_MSTAT_XFER_INP);
}

// Function to write to icm20948 magnetometer, which is seperate from acc/gyro
void mag_write(uint8 regAddress, uint8 data)
{
   
    // Set slave 0 address on the chip's i2c master to the magnetometer's address
    imu_write(I2C_SLV0_ADDR, AK09916_ADDRESS);
   
    // Set the register we want to write to on the mag
    imu_write(I2C_SLV0_REG, regAddress);
   
    // Set data to write
    imu_write(I2C_SLV0_DO, data);
   
    // Send write command
    imu_write(I2C_SLV0_CTRL, 0x81); // Last 1 is important for sending 1 byte!!
}

// Function to read from icm20948 magnetometer, which is seperate from acc/gyro
// Read values are stored in the first available EXT_SENS_DATA register
void mag_read(uint8 regAddress, uint8*bufferPointer, uint8 numBytes)
{
    // Set slave 0 address on the chip's i2c master to the magnetometer's address
    // Add a leading 1 bit to signify read
    imu_write(I2C_SLV0_ADDR, 0x80 | AK09916_ADDRESS);
   
    // Set the register we want to read from on the mag
    imu_write(I2C_SLV0_REG, regAddress);
   
    // Send read command
    // Last 4 bits are set to how many bytes to read
    imu_write(I2C_SLV0_CTRL, 0x80 | numBytes);
   
    // Wait, necessary?
    CyDelay(50);
   
    // Read the external sensor registers
    imu_read(EXT_SLV_SENS_DATA_00,(uint8*)bufferPointer,numBytes);   
}

// Function to get icm20948 acceleration scale factor (LSB/g)
uint16 getAccScaleFac(uint8 accScale)
{
    switch (accScale) {
        case 0: return 16386u;
        case 1: return 8192u;
        case 2: return 4096u;
        case 3: return 2048u;
        default: return 0;
    }
}

// Function to get icm20948 gyroscope scale factor (LSB/dps)
// Returns float due to required decimal points
float getGyroScaleFac(uint8 gyroScale)
{
    switch (gyroScale) {
        case 0: return 131;
        case 1: return 65.5;
        case 2: return 32.8;
        case 3: return 16.4;
        default: return 0;
    }
}

// Function that returns the 2s complement value of a uint16 in float form, used for icm20948
float uint16ToFloat(uint16 val)
{
    // Check msb to see if negative
    if( val & 0x8000)
    {
        return (float)val - 2*pow(2,15);
    }
    return (float)val;
}

// Function to convert float to uint16 (2s complement), used for icm20948
uint16 floatToUint16(float val)
{
    uint16 i;
    // Make positive for conversion to int
    if( val < 0 )
    {
        i = (uint16)(val*-1);
    }
    else
    {
        i = (uint16)val;
    }
    // If val was negative, fix up 2s complement
    // Flip all bits and add 1
    if( val < 0 )
    {
        i = ~i + 1;
    }
    return i;
}

// Function to convert a 15 bit integer (stored as 15b-0) to a 16 bit integer, used for acc offset reg in icm20948
// 2s complement considered
uint16 conv15bto16b(uint16 var)
{
    // Check leading bit to see if negative
    if( var & 0x8000)
    {
        // First bit is a 1, so it's negative
        // Shift everything forward and place a 1 in leading position
        return (var>>1) | 0x8000;
    }
    // Otherwise, it's positive and can simply shift forward 1
    return var>>1;
}

// Timing functions, but only for the timeout timer (for some reason, lol)
uint16 toutTimerStart()
{
    // First reset timer, then return initial time
    ToutTimerResetReg_Write(1); // Turn timer off
    CyDelay(5); // Wait some clock cycles
    ToutTimerResetReg_Write(0); // Turn timer on     
    return ToutTimer_ReadCounter(); // Return initial time (uint16)
}
uint16 toutTimerGetTime(uint16 initTime)
{
    return initTime - ToutTimer_ReadCounter(); // uint16
}

// Function to initialise the icm20948
// Scales can be set to 0,1,2 or 3
// offsetVals is how many readings are averaged to set acc and gyro offset registers
void imu_init(uint8 accScale,uint8 gyroScale, uint16 offsetVals)
{   
    // First set to bank 0, so we are sure that it's actually at 0 (should reset to 0 anyway)
    imu_sel_bank(0 << 4);
   
    // Reset chip
    imu_write( PWR_MGMT_1, 0xC1); // Set reset bit (autoclears), sleep bit and default clock bit
    CyDelay(100);
   
    // Turn on chip (clear sleep bit)
    imu_write( PWR_MGMT_1, 0x01);
    CyDelay(20);
    
    // Enable ODR start time alignment
    imu_write( ODR_ALIGN_EN, 0x01);
    CyDelay(20);
   
    // Set acc and gyro scales, maintains lowpass filters
    imu_write(ACCEL_CONFIG, (accScale << 1) | 0x01 );
    CyDelay(20);
    imu_write(GYRO_CONFIG_1, (gyroScale << 1) | 0x01 );
    CyDelay(20);
    
    // Reset and then enable I2C master
    imu_write(USER_CTRL,0x02);
    CyDelay(100);
    imu_write(USER_CTRL, 0x20);
    CyDelay(10);
   
    // Set I2C master clock frequency to 400 kHz
    imu_write(I2C_MST_CTRL, 0x07);
    CyDelay(10);
    
    // Reset magnetometer and wait... If after offset setting, this resets za!!
    mag_write(MAG_CNTL3, 0x01);
    CyDelay(100);

    // Turn on mag and set to 100Hz... If after offset setting, this resets za!! (either one will, or in combination - mag write responsible?)
    mag_write(MAG_CNTL2, 0x08);
    CyDelay(100);
    
    uint8 dummy[8]; // Used for initial mag_read call
    // Do first read from magnetometer, after which it's read automatically at rate determined by gyro ODR, and stored in ext sensor regs
    mag_read(MAG_HXL,(uint8*)dummy,8);
    
    // Offset calculation and setting
    uint8 dataBuffer[12]; // Buffer to store acc and gyro values
    uint8 accReset[2]; // Buffer to store original acc values (does not reset to 0). Can only read 2 at a time as they regs not sequential
    imu_read(XA_OFFS_H, (uint8*)accReset, 2); // Grab original acc vals
    uint16 ax0 = conv15bto16b(accReset[0]<<8 | accReset[1]); // Combine h/l bytes, convert to 16b, convert to float
    imu_read(YA_OFFS_H, (uint8*)accReset, 2);
    uint16 ay0 = conv15bto16b(accReset[0]<<8 | accReset[1]);
    imu_read(ZA_OFFS_H, (uint8*)accReset, 2);
    uint16 az0 = conv15bto16b(accReset[0]<<8 | accReset[1]);
    // Variables for summing
    float accSum[3] = {0,0,0}; // x,y,z
    float gyroSum[3] = {0,0,0};
    for( uint8 i=0; i<offsetVals; i++)
    {
        // Read accel, gyro registers
        imu_read(ACCEL_XOUT_H, (uint8*)dataBuffer, 12);
        // Add to sum. Convert to float first.
        accSum[0] += uint16ToFloat(dataBuffer[0]<<8 | dataBuffer[1]);
        accSum[1] += uint16ToFloat(dataBuffer[2]<<8 | dataBuffer[3]);
        accSum[2] += uint16ToFloat(dataBuffer[4]<<8 | dataBuffer[5]);
        gyroSum[0] += uint16ToFloat(dataBuffer[6]<<8 | dataBuffer[7]);
        gyroSum[1] += uint16ToFloat(dataBuffer[8]<<8 | dataBuffer[9]);
        gyroSum[2] += uint16ToFloat(dataBuffer[10]<<8 | dataBuffer[11]);
        CyDelay(10);
    }
    // Average. First need scale factors
    uint16 accScaleFac = getAccScaleFac(accScale); float gyroScaleFac = getGyroScaleFac(gyroScale);
    float accAv[3] = {accSum[0]/offsetVals/(float)accScaleFac, accSum[1]/offsetVals/(float)accScaleFac, accSum[2]/offsetVals/(float)accScaleFac};
    float gyroAv[3] = {gyroSum[0]/offsetVals/gyroScaleFac, gyroSum[1]/offsetVals/gyroScaleFac, gyroSum[2]/offsetVals/gyroScaleFac};
    // For acceleration, '0.98-mg steps'. How many of these steps do we want? Want 0,0,1 (subtract 1 from z accAv)
    // Need negative because if our accAv is +ve, we need -ve offset
    // Add original offsets as we want to maintain these
    uint16 accOffs[3] = {-floatToUint16(round(accAv[0]*1000/0.98))+ax0, -floatToUint16(round(accAv[1]*1000/0.98))+ay0, -floatToUint16(round((accAv[2]-1)*1000/0.98))+az0};
    // For gyro, step size of 0.0305 dps/lsb
    // Want to round(float), but then convert to uint16
    uint16 gyroOffs[3] = {-floatToUint16(round(gyroAv[0]/0.0305)), -floatToUint16(round(gyroAv[1]/0.0305)), -floatToUint16(round(gyroAv[2]/0.0305))};
    // Write to registers (would multi-write be better?)
    // Acc
    imu_write(XA_OFFS_H, (accOffs[0]&0x7F80)>>7); // High byte
    imu_write(XA_OFFS_L, (accOffs[0]<<1)&0xFE); // Low byte shifted up 1 and high byte masked off
    imu_write(YA_OFFS_H, (accOffs[1]&0x7F80)>>7);
    imu_write(YA_OFFS_L, (accOffs[1]<<1)&0xFE);
    imu_write(ZA_OFFS_H, (accOffs[2]&0x7F80)>>7);
    imu_write(ZA_OFFS_L, (accOffs[2]<<1)&0xFE);
    // Gyro
    imu_write(XG_OFFS_H, gyroOffs[0]>>8); // High byte
    imu_write(XG_OFFS_L, gyroOffs[0]&0xFF); // Low byte
    imu_write(YG_OFFS_H, gyroOffs[1]>>8);
    imu_write(YG_OFFS_L, gyroOffs[1]&0xFF);
    imu_write(ZG_OFFS_H, gyroOffs[2]>>8);
    imu_write(ZG_OFFS_L, gyroOffs[2]&0xFF);
}

// Function to get imu data and send over bluetooth NOOO
void imu_getData()
{
    uint8 buffer[22]; // Buffer to store read data
    imu_read(ACCEL_XOUT_H, (uint8*)buffer, 22); // Read accel, gyro and external sensor (mag) registers
    // Send to computer over bt
    for( uint8 i=0; i<22; i++)
    {
        BTUART_WriteTxData(buffer[i]);
        CyDelayUs(60); // One byte takes 69 us (115200 baud)
    }
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    // Initialise interrupts, UARTs, I2C and timer
    LIDARUART_Start();
    BTUART_Start();
    I2C_Start();
    ToutTimer_Start();
    DataTimer_Start(); // 100kHz clock, 1 tick is 100th of a ms. Used to track time of data retreival from IMU
    LoopTimer_Start(); // 1kHz clock (ms). Adds control to how often main loop is cycled. Can track 63 seconds.]
    // Doesn't exactly dictate rate of bt data transmission, as if one loop takes a long time, the next can be executed immediately after
    // But it's a good average

    // Initialise icm20948
    // Change these values to set acc and gyro scales. Can be 0,1,2,3. Scale factors are applied in python
    uint8 const ACC_SCALE = 1u;
    uint8 const GYRO_SCALE = 2u;  
    // Set how many readings will averaged to calculate offsets
    uint16 const OFFS_VALS = 100u;    
    // Initialise icm20948
    imu_init(ACC_SCALE,GYRO_SCALE,OFFS_VALS);

    
    // Main loop
    uint8 status;
    uint8 errorStatus;
    uint8 lidarBuffer; // Sent over bt after it is received
    uint8 i=0u; // Tracks lidar data collection position
    
    uint8 imuBuffer[22];
    uint16 imuSum; // Used to make checksum (which has high bits masked off)
    uint8 imuChecksum; // Sent as last byte
    uint8 const IMU_HEADER = 0x58; // Header value for imu data
    
    uint8 mode = 0u; // Controlls what data is sent, and if we have unlimited reads
    uint8 count = 0u; // Track how many reads have been requested
    uint16 toutInitTime = toutTimerStart(); // For timeout timer
    uint16 const BT_TIMEOUT = 150u; // How many ms will we wait for bt update before shutting off output?
    
    uint16 readTime; // To send over bt, to work out dt
    
    uint16 const MIN_LOOP_TIME = 10u; // How many ms should the loop go for minimum? With a byte delay of 200us, loop lasts ~8ms, setting to
                                      // 10ms keeps sample rate more predictable, works for hc06
    // Hc05 seems to need 20ms (Not even 18 works!), and then acts predictably and can handle byte delay of 100us (but checksum fails occasionally)
    
    uint8 const UART_BYTE_DELAY = 200u; // UART send delay in us, baud is 115200, about 70us is one byte.
                                       // Setting to 60 causes data loss at i2c transfer
                                        // Experimentally, hc05 needs 200us delay between bytes minimum for constant sending. 
                                        // hc06 can handle 100us
    
    uint16 startTime; // What counter value are we up to at start of loop?
    uint16 endTime;
    uint16 loopInterval;
    uint16 const MAX_COUNTER = 0xFFFF; // Max value a uint16 can take, used to find interval if clock cycle restarts
    
    
    uint8 temp; // To store bt data
    
    
    for(;;)
    {
        // Set start time
        startTime = LoopTimer_ReadCounter();
        
        // 1. Check for new bt request
        status = BTUART_ReadRxStatus();
        if((status & BTUART_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            // Evaluate error status
            errorStatus = status & ( BTUART_RX_STS_BREAK      | BTUART_RX_STS_PAR_ERROR | 
                                     BTUART_RX_STS_STOP_ERROR | BTUART_RX_STS_OVERRUN);
            // Continue if no errors
            if(errorStatus == 0u)
            {
                temp = BTUART_ReadRxData();
                mode =  temp & 0x07; // Mask off higher 5 bits - only store continuous read on/off and lidar/imu on/off
                count +=  temp >> COUNT_SHIFT; // Add higher 5 bits to count
                toutInitTime = toutTimerStart(); // Reset timer and get initial time of bt request
            }
        }
        
        // 2. Read number control
        // If count==0 and cont_mode isn't on, turn off output
        if( count==0 && !(mode & CONT_MODE) )
        {
            mode = 0u;
        }

        // 3. Loop over lidar data and send as received, but only if mode is set. Blocking until lidar data is sent
        i = 0; // Reset i
        while( i<9 && (mode & LIDAR_ON) )
        {
            // Check status of lidar Rx
            status = LIDARUART_ReadRxStatus();
            if((status & LIDARUART_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                // Evaluate error status
                errorStatus = status & ( LIDARUART_RX_STS_BREAK      | LIDARUART_RX_STS_PAR_ERROR | 
                                         LIDARUART_RX_STS_STOP_ERROR | LIDARUART_RX_STS_OVERRUN);
                // Continue if no errors
                if(errorStatus == 0u)
                {
                    // Read data
                    lidarBuffer = LIDARUART_ReadRxData();
                    // If 0<i<2 then we should have the frame header. Only want to send it if get both in succession
                    if( i==0u || i==1u )
                    {
                        if( lidarBuffer != 0x59 )
                        {
                            i = 0u; // If no frame header, reset counter and try again
                            continue;
                        }
                        else if(i == 1u)
                        {
                            BTUART_WriteTxData(lidarBuffer); // Send 0x59
                            CyDelayUs(UART_BYTE_DELAY);
                            BTUART_WriteTxData(lidarBuffer); // Send 0x59
                            CyDelayUs(UART_BYTE_DELAY);
                            i++;
                            continue;
                        }
                        else if( i == 0u )
                        {
                            i++;
                            continue;
                        }
                    }
                    // Trailing values 2<=i<=8 (incl checksum)
                    BTUART_WriteTxData(lidarBuffer); // Send byte
                    i++;
                    CyDelayUs(UART_BYTE_DELAY); // Delay between bytes doesn't need to be as long as a byte as time taken in receiving data too?
                }   
            }
        }

        // 4. Get imu data with i2c call, if requested, and send over bt
        if( mode & IMU_ON )
        {
            // Read accel, gyro and external sensor (mag) registers
            imu_read(ACCEL_XOUT_H, (uint8*)imuBuffer, 22);
            
            // Grab time now
            readTime = DataTimer_ReadCounter();
            
            // Send header
            BTUART_WriteTxData(IMU_HEADER);
            CyDelayUs(UART_BYTE_DELAY);
            BTUART_WriteTxData(IMU_HEADER);
            CyDelayUs(UART_BYTE_DELAY);
            
            imuSum = 2*IMU_HEADER; // Have sent two headers so far
            // Send each byte
            for( uint8 j = 0; j<22u; j++ )
            {
                if( j==20 )continue; // Don't send the mag dummy register (reads 0 only)
                imuSum+=imuBuffer[j];
                BTUART_WriteTxData(imuBuffer[j]);
                CyDelayUs(UART_BYTE_DELAY);
            }
            // Add the recorded counter of DataTimer
            imuSum+=readTime>>8; // Add to checksum
            BTUART_WriteTxData(readTime>>8); // High byte
            CyDelayUs(UART_BYTE_DELAY);
            imuSum+=readTime&0xFF;
            BTUART_WriteTxData(readTime&0xFF); // Low byte
            CyDelayUs(UART_BYTE_DELAY);
           
            // Calculate checksum
            imuChecksum = imuSum&0xFF;
            BTUART_WriteTxData(imuChecksum); // Send checksum. A total of 2 header bytes, 21 data bytes, 2 time bytes and 1 checksum have been sent
            CyDelayUs(UART_BYTE_DELAY);
        }
        
        // 5. Read number control part 2
        // Subtract from count if it's larger than 0, and something is on
        if( count > 0u && (mode & (LIDAR_ON|IMU_ON)) ) count -= 1;
            
        
        // 6. Timeout control
        if( toutTimerGetTime(toutInitTime) > BT_TIMEOUT ) // Stop bt if no response within timeout
        {
            // Timer counts down from 63 seconds and then resets to 63 again
            // So if timerStart() isn't called in this time, timerGetTime will be incorrect (too small)
            // This shouldn't cause any problems
            mode = 0u;
        }
        
        // 7. Loop timing control
        // How long did the loop take?
        endTime = LoopTimer_ReadCounter();
        if( endTime < startTime )
        {
            loopInterval = startTime - endTime;
        }
        else // Timer reset to max counter
        {
            loopInterval = startTime + (MAX_COUNTER-endTime);
        }
        // Do we need to add delay?
        if( loopInterval < MIN_LOOP_TIME )
        {
            CyDelay(MIN_LOOP_TIME - loopInterval);
        }
    }
}

/* [] END OF FILE */
