#include <peripheral/i2c.h>
#include <peripheral/wdt.h>
#include <stdlib.h>
#include "APDS9960.h"
#include "SSD1306.h"

static I2C_7_BIT_ADDRESS SlaveAddress;
static UINT8 i2cData[24];
static UINT8 convertBuf[8];
static BOOL debug = FALSE;

static BOOL wireWriteByte(UINT8 val);
static BOOL wireWriteDataByte(UINT8 reg, UINT8 val);
static BOOL wireWriteDataBlock(UINT8 reg, UINT8 *val, UINT len);
static BOOL wireReadDataByte(UINT8 reg, UINT8 *val);
static UINT wireReadDataBlock(UINT8 reg, UINT8 *val, UINT len);

/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/

/**
 * @brief Writes a single byte to the I2C device (no register)
 *
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
BOOL wireWriteByte(UINT8 val)
{
//    MyPrint("Start byte writing.\n");
    
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, APDS9960_I2C_ADDR, I2C_WRITE);
    UINT8 nBytes = 2;
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = val;                   // Data byte
    
    // Start the transfer to write data
    if( !StartTransfer(FALSE) )
    {
        MyPrint("Start Transfer Error.\n");
    }

    // Transmit all data
    int Index = 0;
    BOOL Success = TRUE;
    while( Success && (Index < nBytes) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(I2C_BUS))
            {
                MyPrint("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }

    // End the transfer hang here if an error occured)
    StopTransfer();
    if(!Success)
    {
        MyPrint("End Transfer Error.\n");
    }


    // Wait for I2C to complete write process, by polling the ack status.
    BOOL Acknowledged = FALSE;
    do
    {
        // Start the transfer to address the EEPROM
        if( !StartTransfer(FALSE) )
        {
            MyPrint("Start Transfer Error.\n");
        }

        // Transmit just the EEPROM's address
        if (TransmitOneByte(SlaveAddress.byte))
        {
            // Check to see if the byte was acknowledged
            Acknowledged = I2CByteWasAcknowledged(I2C_BUS);
        }
        else
        {
            Success = FALSE;
        }

        // End the transfer (stop here if an error occured)
        StopTransfer();
        if(!Success)
        {
            MyPrint("End Transfer Error.\n");
        }

    } while (Acknowledged != TRUE);
    return Success;
}

/**
 * @brief Writes a single byte to the I2C device and specified register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
BOOL wireWriteDataByte(UINT8 reg, UINT8 val)
{
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, APDS9960_I2C_ADDR, I2C_WRITE);
    UINT8 nBytes = 3;
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = reg;
    i2cData[2] = val;                   // Data byte
    
    // Start the transfer to write data
    if( !StartTransfer(FALSE) )
    {
        MyPrint("Start Transfer Error.\n");
    }

    // Transmit all data
    int Index = 0;
    BOOL Success = TRUE;
    while( Success && (Index < nBytes) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(I2C_BUS))
            {
                MyPrint("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }

    // End the transfer (hang here if an error occured)
    StopTransfer();
    if(!Success)
    {
        MyPrint("End Transfer Error.\n");
    }


    // Wait for I2C to complete write process, by polling the ack status.
    BOOL Acknowledged = FALSE;
    do
    {
        // Start the transfer to address the EEPROM
        if( !StartTransfer(FALSE) )
        {
            MyPrint("Start Transfer Error.\n");
        }

        // Transmit just the EEPROM's address
        if (TransmitOneByte(SlaveAddress.byte))
        {
            // Check to see if the byte was acknowledged
            Acknowledged = I2CByteWasAcknowledged(I2C_BUS);
        }
        else
        {
            Success = FALSE;
        }

        // End the transfer (stop here if an error occured)
        StopTransfer();
        if(!Success)
        {
            MyPrint("End Transfer Error.\n");
        }

    } while (Acknowledged != TRUE);
    return Success;
}

/**
 * @brief Writes a block (array) of bytes to the I2C device and register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val pointer to the beginning of the data byte array
 * @param[in] len the length (in bytes) of the data to write
 * @return True if successful write operation. False otherwise.
 */
BOOL wireWriteDataBlock(UINT8 reg, UINT8 *val, UINT len)
{
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, APDS9960_I2C_ADDR, I2C_WRITE);
    UINT8 nBytes = len+2;
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = reg;
    UINT i;
    for(i = 0; i < len; i++){
        i2cData[2+i] = *(val+i);
    }               
    
    // Start the transfer to write data
    if( !StartTransfer(FALSE) )
    {
        MyPrint("Start Transfer Error.\n");
    }

    // Transmit all data
    int Index = 0;
    BOOL Success = TRUE;
    while( Success && (Index < nBytes) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(I2C_BUS))
            {
                MyPrint("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }

    // End the transfer (hang here if an error occured)
    StopTransfer();
    if(!Success)
    {
        MyPrint("End Transfer Error.\n");
    }


    // Wait for I2C to complete write process, by polling the ack status.
    BOOL Acknowledged = FALSE;
    do
    {
        // Start the transfer to address the EEPROM
        if( !StartTransfer(FALSE) )
        {
            MyPrint("Start Transfer Error.\n");
        }

        // Transmit just the EEPROM's address
        if (TransmitOneByte(SlaveAddress.byte))
        {
            // Check to see if the byte was acknowledged
            Acknowledged = I2CByteWasAcknowledged(I2C_BUS);
        }
        else
        {
            Success = FALSE;
        }

        // End the transfer (stop here if an error occured)
        StopTransfer();
        if(!Success)
        {
            MyPrint("End Transfer Error.\n");
        }

    } while (Acknowledged != TRUE);
    return Success;
}

/**
 * @brief Reads a single byte from the I2C device and specified register
 *
 * @param[in] reg the register to read from
 * @param[out] the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
BOOL wireReadDataByte(UINT8 reg, UINT8 *val)
{   
    /* Indicate which register we want to read from */
    if (!wireWriteByte(reg)) {
        return FALSE;
    }

    BOOL Success = TRUE;
    // Send a Repeated Started condition
    if( !StartTransfer(TRUE) )
    {
        MyPrint("Start Transfer Error.\n");
    }

    // Transmit the address with the READ bit set
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, APDS9960_I2C_ADDR, I2C_READ);
    if (TransmitOneByte(SlaveAddress.byte))
    {
        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(I2C_BUS))
        {
            MyPrint("Error: Sent byte was not acknowledged\n");
            Success = FALSE;
        }
    }
    else
    {
        MyPrint("TransferOneByte Error.\n");
        Success = FALSE;
    }
    
    if(Success)
    {
        if(debug == TRUE)
            MyPrint("DEBUG3.\n");
        if(I2CReceiverEnable(I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
        {
            MyPrint("Error: I2C Receive Overflow\n");
            Success = FALSE;
        }
        else
        {
            while(!I2CReceivedDataIsAvailable(I2C_BUS)){if(debug == TRUE)MyPrint("+");};
            BYTE byte = I2CGetByte(I2C_BUS);

            *val = byte;
        }

    }

    // End the transfer (stop here if an error occured)
    StopTransfer();
    if(!Success)
    {
        MyPrint("End Transfer Error.\n");
    }

    return Success;
}

/**
 * @brief Reads a block (array) of bytes from the I2C device and register
 *
 * @param[in] reg the register to read from
 * @param[out] val pointer to the beginning of the data
 * @param[in] len number of bytes to read
 * @return Number of bytes read. -1 on read error.
 */
UINT wireReadDataBlock(UINT8 reg, UINT8 *val, UINT len)
{
    /* Indicate which register we want to read from */
    if (!wireWriteByte(reg)) {
        return FALSE;
    }

    BOOL Success = TRUE;
    // Send a Repeated Started condition
    if( !StartTransfer(TRUE) )
    {
        MyPrint("Start Transfer Error.\n");
    }

    // Transmit the address with the READ bit set
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, APDS9960_I2C_ADDR, I2C_READ);
    if (TransmitOneByte(SlaveAddress.byte))
    {
        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(I2C_BUS))
        {
            MyPrint("Error: Sent byte was not acknowledged\n");
            Success = FALSE;
        }
    }
    else
    {
        MyPrint("TransferOneByte Error.\n");
        Success = FALSE;
    }
    
    UINT8 i = 0;
    // Read the data from the desired address
    if(Success)
    {
        if(I2CReceiverEnable(I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
        {
            MyPrint("Error: I2C Receive Overflow\n");
            Success = FALSE;
        }
        else
        {
            for(i = 0; i < len; i++){
//                UINT8 length = convertByteToCharacter(i, convertBuf, 24);
//                SendDataBuffer(convertBuf + 24 - length, length);
//                MyPrint("\n");
                while(!I2CReceivedDataIsAvailable(I2C_BUS));
                if(i != len-1){
                    I2CAcknowledgeByte(I2C_BUS, TRUE);
                    while(!I2CAcknowledgeHasCompleted(I2C_BUS));
                    I2CReceiverEnable(I2C_BUS, TRUE);
                }
                UINT8 byte = I2CGetByte(I2C_BUS);
                *(val+i) = byte;
            }
//            MyPrint("----\n");
        }

    }

    // End the transfer (stop here if an error occured)
    StopTransfer();
    if(!Success)
    {
        MyPrint("End Transfer Error.\n");
    }

    return i;
}

BOOL initAPDS9960(APDS9960_t *pApds){
    pApds->gesture_ud_delta_ = 0;
    pApds->gesture_lr_delta_ = 0;
    
    pApds->gesture_ud_count_ = 0;
    pApds->gesture_lr_count_ = 0;
    
    pApds->gesture_near_count_ = 0;
    pApds->gesture_far_count_ = 0;
    
    pApds->gesture_state_ = 0;
    pApds->gesture_motion_ = DIR_NONE;
    
    UINT8 id;
    
    /* Read ID register and check against known values for APDS-9960 */
    if( !wireReadDataByte(APDS9960_ID, &id) ) {
        return FALSE;
    }
    if( !(id == APDS9960_ID_1 || id == APDS9960_ID_2) ) {
        return FALSE;
    }

    /* Set ENABLE register to 0 (disable all features) */
    if( !setMode(ALL, OFF) ) {
        return FALSE;
    }
    
    /* Set default values for ambient light and proximity registers */
    if( !wireWriteDataByte(APDS9960_ATIME, DEFAULT_ATIME) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_WTIME, DEFAULT_WTIME) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_CONFIG1, DEFAULT_CONFIG1) ) {
        return FALSE;
    }
    if( !setLEDDrive(DEFAULT_LDRIVE) ) {
        return FALSE;
    }
    if( !setProximityGain(DEFAULT_PGAIN) ) {
        return FALSE;
    }
    if( !setAmbientLightGain(DEFAULT_AGAIN) ) {
        return FALSE;
    }
    if( !setProxIntLowThresh(DEFAULT_PILT) ) {
        return FALSE;
    }
    if( !setProxIntHighThresh(DEFAULT_PIHT) ) {
        return FALSE;
    }
    if( !setLightIntLowThreshold(DEFAULT_AILT) ) {
        return FALSE;
    }
    if( !setLightIntHighThreshold(DEFAULT_AIHT) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_PERS, DEFAULT_PERS) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_CONFIG2, DEFAULT_CONFIG2) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_CONFIG3, DEFAULT_CONFIG3) ) {
        return FALSE;
    }
    
    /* Set default values for gesture sense registers */
    if( !setGestureEnterThresh(DEFAULT_GPENTH) ) {
        return FALSE;
    }
    if( !setGestureExitThresh(DEFAULT_GEXTH) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_GCONF1, DEFAULT_GCONF1) ) {
        return FALSE;
    }
    if( !setGestureGain(DEFAULT_GGAIN) ) {
        return FALSE;
    }
    if( !setGestureLEDDrive(DEFAULT_GLDRIVE) ) {
        return FALSE;
    }
    if( !setGestureWaitTime(DEFAULT_GWTIME) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_GPULSE, DEFAULT_GPULSE) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_GCONF3, DEFAULT_GCONF3) ) {
        return FALSE;
    }
    if( !setGestureIntEnable(DEFAULT_GIEN) ) {
        return FALSE;
    }
    
    return TRUE;
}

/*******************************************************************************
 * Public methods for controlling the APDS-9960
 ******************************************************************************/

/**
 * @brief Reads and returns the contents of the ENABLE register
 *
 * @return Contents of the ENABLE register. 0xFF if error.
 */
UINT8 getMode()
{
    UINT8 enable_value;
    
    /* Read current ENABLE register */
    if( !wireReadDataByte(APDS9960_ENABLE, &enable_value) ) {
        return ERROR;
    }
    
    return enable_value;
}

/**
 * @brief Enables or disables a feature in the APDS-9960
 *
 * @param[in] mode which feature to enable
 * @param[in] enable ON (1) or OFF (0)
 * @return TRUE if operation success. FALSE otherwise.
 */
BOOL setMode(UINT8 mode, UINT8 enable)
{
    UINT8 reg_val;

    /* Read current ENABLE register */
    reg_val = getMode();
    if( reg_val == ERROR ) {
        return FALSE;
    }
    
    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if( mode >= 0 && mode <= 6 ) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }
        
    /* Write value back to ENABLE register */
    if( !wireWriteDataByte(APDS9960_ENABLE, reg_val) ) {
        return FALSE;
    }
        
    return TRUE;
}

/**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value (0-3) for the LED drive strength
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setLEDDrive(UINT8 drive)
{
    UINT8 val;
    
    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9960_CONTROL, &val) ) {
        return FALSE;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 6;
    val &= 0b00111111;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9960_CONTROL, val) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setProximityGain(UINT8 drive)
{
    UINT8 val;
    
    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9960_CONTROL, &val) ) {
        return FALSE;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 2;
    val &= 0b11110011;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9960_CONTROL, val) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setAmbientLightGain(UINT8 drive)
{
    UINT8 val;
    
    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9960_CONTROL, &val) ) {
        return FALSE;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    val &= 0b11111100;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9960_CONTROL, val) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] threshold the lower proximity threshold
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setProxIntLowThresh(UINT8 threshold)
{
    if( !wireWriteDataByte(APDS9960_PILT, threshold) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setProxIntHighThresh(UINT8 threshold)
{
    if( !wireWriteDataByte(APDS9960_PIHT, threshold) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the low threshold for ambient light interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setLightIntLowThreshold(UINT16 threshold)
{
    UINT8 val_low;
    UINT8 val_high;
    
    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;
    
    /* Write low byte */
    if( !wireWriteDataByte(APDS9960_AILTL, val_low) ) {
        return FALSE;
    }
    
    /* Write high byte */
    if( !wireWriteDataByte(APDS9960_AILTH, val_high) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the high threshold for ambient light interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setLightIntHighThreshold(UINT16 threshold)
{
    UINT8 val_low;
    UINT8 val_high;
    
    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;
    
    /* Write low byte */
    if( !wireWriteDataByte(APDS9960_AIHTL, val_low) ) {
        return FALSE;
    }
    
    /* Write high byte */
    if( !wireWriteDataByte(APDS9960_AIHTH, val_high) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the entry proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to start gesture mode
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setGestureEnterThresh(UINT8 threshold)
{
    if( !wireWriteDataByte(APDS9960_GPENTH, threshold) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the exit proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to end gesture mode
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setGestureExitThresh(UINT8 threshold)
{
    if( !wireWriteDataByte(APDS9960_GEXTH, threshold) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] gain the value for the photodiode gain
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setGestureGain(UINT8 gain)
{
    UINT8 val;
    
    /* Read value from GCONF2 register */
    if( !wireReadDataByte(APDS9960_GCONF2, &val) ) {
        return FALSE;
    }
    
    /* Set bits in register to given value */
    gain &= 0b00000011;
    gain = gain << 5;
    val &= 0b10011111;
    val |= gain;
    
    /* Write register value back into GCONF2 register */
    if( !wireWriteDataByte(APDS9960_GCONF2, val) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the LED drive current during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value for the LED drive current
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setGestureLEDDrive(UINT8 drive)
{
    UINT8 val;
    
    /* Read value from GCONF2 register */
    if( !wireReadDataByte(APDS9960_GCONF2, &val) ) {
        return FALSE;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 3;
    val &= 0b11100111;
    val |= drive;
    
    /* Write register value back into GCONF2 register */
    if( !wireWriteDataByte(APDS9960_GCONF2, val) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] the value for the wait time
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setGestureWaitTime(UINT8 time)
{
    UINT8 val;
    
    /* Read value from GCONF2 register */
    if( !wireReadDataByte(APDS9960_GCONF2, &val) ) {
        return FALSE;
    }
    
    /* Set bits in register to given value */
    time &= 0b00000111;
    val &= 0b11111000;
    val |= time;
    
    /* Write register value back into GCONF2 register */
    if( !wireWriteDataByte(APDS9960_GCONF2, val) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Turns gesture-related interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setGestureIntEnable(UINT8 enable)
{
    UINT8 val;
    
    /* Read value from GCONF4 register */
    if( !wireReadDataByte(APDS9960_GCONF4, &val) ) {
        return FALSE;
    }
    
    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 1;
    val &= 0b11111101;
    val |= enable;
    
    /* Write register value back into GCONF4 register */
    if( !wireWriteDataByte(APDS9960_GCONF4, val) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Starts the gesture recognition engine on the APDS-9960
 *
 * @param[in] interrupts TRUE to enable hardware external interrupt on gesture
 * @return TRUE if engine enabled correctly. FALSE on error.
 */
BOOL enableGestureSensor(APDS9960_t *pApds, BOOL interrupts)
{
    
    /* Enable gesture mode
       Set ENABLE to 0 (power off)
       Set WTIME to 0xFF
       Set AUX to LED_BOOST_300
       Enable PON, WEN, PEN, GEN in ENABLE 
    */
    resetGestureParameters(pApds);
    if( !wireWriteDataByte(APDS9960_WTIME, 0xFF) ) {
        return FALSE;
    }
    if( !wireWriteDataByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE) ) {
        return FALSE;
    }
    if( !setLEDBoost(LED_BOOST_300) ) {
        return FALSE;
    }
    if( interrupts ) {
        if( !setGestureIntEnable(1) ) {
            return FALSE;
        }
    } else {
        if( !setGestureIntEnable(0) ) {
            return FALSE;
        }
    }
    if( !setGestureMode(1) ) {
        return FALSE;
    }
    if( !enablePower() ){
        return FALSE;
    }
    if( !setMode(WAIT, 1) ) {
        return FALSE;
    }
    if( !setMode(PROXIMITY, 1) ) {
        return FALSE;
    }
    if( !setMode(GESTURE, 1) ) {
        return FALSE;
    }
    
    return TRUE;
}

/*******************************************************************************
 * High-level gesture controls
 ******************************************************************************/

/**
 * @brief Resets all the parameters in the gesture data member
 */
void resetGestureParameters(APDS9960_t *pApds)
{
    pApds->gesture_data_.index = 0;
    pApds->gesture_data_.total_gestures = 0;
    
    pApds->gesture_ud_delta_ = 0;
    pApds->gesture_lr_delta_ = 0;
    
    pApds->gesture_ud_count_ = 0;
    pApds->gesture_lr_count_ = 0;
    
    pApds->gesture_near_count_ = 0;
    pApds->gesture_far_count_ = 0;
    
    pApds->gesture_state_ = 0;
    pApds->gesture_motion_ = DIR_NONE;
}

/**
 * @brief Sets the LED current boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setLEDBoost(UINT8 boost)
{
    UINT8 val;
    
    /* Read value from CONFIG2 register */
    if( !wireReadDataByte(APDS9960_CONFIG2, &val) ) {
        return FALSE;
    }
    
    /* Set bits in register to given value */
    boost &= 0b00000011;
    boost = boost << 4;
    val &= 0b11001111;
    val |= boost;
    
    /* Write register value back into CONFIG2 register */
    if( !wireWriteDataByte(APDS9960_CONFIG2, val) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * @brief Tells the state machine to either enter or exit gesture state machine
 *
 * @param[in] mode 1 to enter gesture state machine, 0 to exit.
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL setGestureMode(UINT8 mode)
{
    UINT8 val;
    
    /* Read value from GCONF4 register */
    if( !wireReadDataByte(APDS9960_GCONF4, &val) ) {
        return FALSE;
    }
    
    /* Set bits in register to given value */
    mode &= 0b00000001;
    val &= 0b11111110;
    val |= mode;
    
    /* Write register value back into GCONF4 register */
    if( !wireWriteDataByte(APDS9960_GCONF4, val) ) {
        return FALSE;
    }
    
    return TRUE;
}

/**
 * Turn the APDS-9960 on
 *
 * @return TRUE if operation successful. FALSE otherwise.
 */
BOOL enablePower()
{
    if( !setMode(POWER, 1) ) {
        return FALSE;
    }
    
    return TRUE;
}

extern UINT8 displayTheme;
extern UINT8 maxTheme;
extern BOOL themeChange;
extern UINT8 countForRefresh;
extern BOOL waitForRefresh;

int handleGesture(APDS9960_t *pApds, UINT8 loopCounter){
    if ( isGestureAvailable() ) {
        int gesture = readGesture(pApds, loopCounter);
        if(gesture <= 3 && waitForRefresh == FALSE){
            waitForRefresh = TRUE;
            countForRefresh = 0;
        }
        else if(waitForRefresh == TRUE){
            MyPrint("WAIT!!");
        }
        switch ( gesture ) {
        case DIR_UP:
            MyPrint("UP\n");
            themeChange = TRUE;
            displayArrow(1);
            break;
        case DIR_DOWN:
            MyPrint("DOWN\n");
            themeChange = TRUE;
            displayArrow(2);
            break;
        case DIR_LEFT:
            MyPrint("LEFT\n");
            themeChange = TRUE;
            displayArrow(4);
            break;
        case DIR_RIGHT:
            MyPrint("RIGHT\n");
//            displayTheme++;
//            displayTheme = displayTheme % maxTheme;
            themeChange = TRUE;
            displayArrow(3);
            break;
        case DIR_NEAR:
            MyPrint("NEAR\n");
            break;
        case DIR_FAR:
            MyPrint("FAR\n");
            break;
        case DIR_NONE:
            MyPrint("NONE\n");
            break;
        default:
            MyPrint("ERROR\n");
            break;
        }
        return gesture;
    } else {
        return DIR_NONE;
    }
}

/**
 * @brief Determines if there is a gesture available for reading
 *
 * @return TRUE if gesture available. FALSE otherwise.
 */
BOOL isGestureAvailable()
{
    UINT8 val;
    
    /* Read value from GSTATUS register */
    if( !wireReadDataByte(APDS9960_GSTATUS, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out GVALID bit */
    val &= APDS9960_GVALID;
    
    /* Return TRUE/FALSE based on GVALID bit */
    if( val == 1) {
        return TRUE;
    } else {
        return FALSE;
    }
}


static UINT8 fifo_level = 0;
static UINT8 bytes_read = 0;
static UINT8 fifo_data[128];
static UINT8 gstatus;
static int motion;
static int i;
static const UINT8 maxTry = 100;
static UINT8 try = 0;

/**
 * @brief Processes a gesture event and returns best guessed gesture
 *
 * @return Number corresponding to gesture. -1 on error.
 */
int readGesture(APDS9960_t *pApds, UINT8 loopCounter)
{   
    /* Make sure that power and gesture is on and data is valid */
    if( !isGestureAvailable() || !(getMode() & 0b01000001) ) {
        return DIR_NONE;
    }
   
    /* Keep looping as long as gesture data is valid */
    while(1) {
//        debug = TRUE;
        /* Wait some time to collect next batch of FIFO data */
        DelayMs(FIFO_PAUSE_TIME);
        
        /* Get the contents of the STATUS register. Is data still valid? */
        if( !wireReadDataByte(APDS9960_GSTATUS, &gstatus) ) {
            MyPrint("Error.\n");
            return ERROR;
        }
        
        /* If we have valid data, read in FIFO */
        
        if( (gstatus & APDS9960_GVALID) == APDS9960_GVALID ) {
        
            /* Read the current FIFO level */
            if( !wireReadDataByte(APDS9960_GFLVL, &fifo_level) ) {
                return ERROR;
            }

//            try++;
//            if(try == maxTry){
//                return DIR_NONE;
//            }
#if DEBUG
            MyPrint("FIFO Level: ");
            MyPrint(fifo_level);
            MyPrint("\n");
#endif

            /* If there's stuff in the FIFO, read it into our data block */
            if( fifo_level > 0) {
//                try = 0;
//                MyPrint("*");
//                MyPrint("Read Gesture: \n");
                bytes_read = wireReadDataBlock( APDS9960_GFIFO_U, 
                                                fifo_data, 
                                                fifo_level*4);
//                MyPrint("Read completed.\n");
                if( bytes_read == -1 ) {
                    return ERROR;
                }
#if DEBUG
                MyPrint("FIFO Dump: ");
                for ( i = 0; i < bytes_read; i++ ) {
                    MyPrint(fifo_data[i]);
                    MyPrint(" ");
                }
                MyPrint("\n");
#endif

                /* If at least 1 set of data, sort the data into U/D/L/R */
                if( bytes_read >= 4 ) {
                    for( i = 0; i < bytes_read; i += 4 ) {
                        //MyPrint(".");
                        pApds->gesture_data_.u_data[pApds->gesture_data_.index] = \
                                                            fifo_data[i + 0];
                        pApds->gesture_data_.d_data[pApds->gesture_data_.index] = \
                                                            fifo_data[i + 1];
                        pApds->gesture_data_.l_data[pApds->gesture_data_.index] = \
                                                            fifo_data[i + 2];
                        pApds->gesture_data_.r_data[pApds->gesture_data_.index] = \
                                                            fifo_data[i + 3];
                        pApds->gesture_data_.index++;
                        pApds->gesture_data_.total_gestures++;
                    }
                
                
#if DEBUG
                    MyPrint("Up Data: ");
                    for ( i = 0; i < pApds->gesture_data_.total_gestures; i++ ) {
                        MyPrint(pApds->gesture_data_.u_data[i]);
                        MyPrint(" ");
                    }
                    MyPrint("\n");
#endif

                    /* Filter and process gesture data. Decode near/far state */
                    if( processGestureData(pApds) ) {
                        if( decodeGesture(pApds) ) {
                            //***TODO: U-Turn Gestures
#if DEBUG
                            //MyPrint(pApds->gesture_motion_);
#endif
                        }
                    }
                    
                    /* Reset data */
                    pApds->gesture_data_.index = 0;
                    pApds->gesture_data_.total_gestures = 0;
                }

            }
            else {
//                MyPrint("Decode data.\n");
                /* Determine best guessed gesture and clean up */

                decodeGesture(pApds);
                motion = pApds->gesture_motion_;

                resetGestureParameters(pApds);

                return motion;
            }
        } else {
    
            
            DelayMs(FIFO_PAUSE_TIME);
            
//            MyPrint("Decode data.\n");
            /* Determine best guessed gesture and clean up */
            decodeGesture(pApds);
            motion = pApds->gesture_motion_;
#if DEBUG
            MyPrint("END: ");
            MyPrint(pApds->gesture_motion_);
#endif
            resetGestureParameters(pApds);
            return motion;
        }
    }
}

/**
 * @brief Processes the raw gesture data to determine swipe direction
 *
 * @return TRUE if near or far state seen. FALSE otherwise.
 */
BOOL processGestureData(APDS9960_t *pApds)
{
    UINT8 u_first = 0;
    UINT8 d_first = 0;
    UINT8 l_first = 0;
    UINT8 r_first = 0;
    UINT8 u_last = 0;
    UINT8 d_last = 0;
    UINT8 l_last = 0;
    UINT8 r_last = 0;
    int ud_ratio_first;
    int lr_ratio_first;
    int ud_ratio_last;
    int lr_ratio_last;
    int ud_delta;
    int lr_delta;
    int i;

    /* If we have less than 4 total gestures, that's not enough */
    if( pApds->gesture_data_.total_gestures <= 4 ) {
        return FALSE;
    }
    
    /* Check to make sure our data isn't out of bounds */
    if( (pApds->gesture_data_.total_gestures <= 32) && \
        (pApds->gesture_data_.total_gestures > 0) ) {
        
        /* Find the first value in U/D/L/R above the threshold */
        for( i = 0; i < pApds->gesture_data_.total_gestures; i++ ) {
            if( (pApds->gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (pApds->gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (pApds->gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (pApds->gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
                
                u_first = pApds->gesture_data_.u_data[i];
                d_first = pApds->gesture_data_.d_data[i];
                l_first = pApds->gesture_data_.l_data[i];
                r_first = pApds->gesture_data_.r_data[i];
                break;
            }
        }
        
        /* If one of the _first values is 0, then there is no good data */
        if( (u_first == 0) || (d_first == 0) || \
            (l_first == 0) || (r_first == 0) ) {
            
            return FALSE;
        }
        /* Find the last value in U/D/L/R above the threshold */
        for( i = pApds->gesture_data_.total_gestures - 1; i >= 0; i-- ) {
#if DEBUG
            MyPrint("Finding last: ");
            MyPrint("U:");
            MyPrint(pApds->gesture_data_.u_data[i]);
            MyPrint(" D:");
            MyPrint(pApds->gesture_data_.d_data[i]);
            MyPrint(" L:");
            MyPrint(pApds->gesture_data_.l_data[i]);
            MyPrint(" R:");
            MyPrint(pApds->gesture_data_.r_data[i]);
            MyPrint("\n");
#endif
            if( (pApds->gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (pApds->gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (pApds->gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (pApds->gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
                
                u_last = pApds->gesture_data_.u_data[i];
                d_last = pApds->gesture_data_.d_data[i];
                l_last = pApds->gesture_data_.l_data[i];
                r_last = pApds->gesture_data_.r_data[i];
                break;
            }
        }
    }
    
    /* Calculate the first vs. last ratio of up/down and left/right */
    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);
       
#if DEBUG
    MyPrint("Last Values: ");
    MyPrint("U:");
    MyPrint(u_last);
    MyPrint(" D:");
    MyPrint(d_last);
    MyPrint(" L:");
    MyPrint(l_last);
    MyPrint(" R:");
    MyPrint(r_last);
    MyPrint("\n");

    MyPrint("Ratios: ");
    MyPrint("UD Fi: ");
    MyPrint(ud_ratio_first);
    MyPrint(" UD La: ");
    MyPrint(ud_ratio_last);
    MyPrint(" LR Fi: ");
    MyPrint(lr_ratio_first);
    MyPrint(" LR La: ");
    MyPrint(lr_ratio_last);
    MyPrint("\n");
#endif
       
    /* Determine the difference between the first and last ratios */
    ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;
    
#if DEBUG
    MyPrint("Deltas: ");
    MyPrint("UD: ");
    MyPrint(ud_delta);
    MyPrint(" LR: ");
    MyPrint(lr_delta);
#endif

    /* Accumulate the UD and LR delta values */
    pApds->gesture_ud_delta_ += ud_delta;
    pApds->gesture_lr_delta_ += lr_delta;
    
#if DEBUG
    MyPrint("Accumulations: ");
    MyPrint("UD: ");
    MyPrint(pApds->gesture_ud_delta_);
    MyPrint(" LR: ");
    MyPrint(pApds->gesture_lr_delta_);
    MyPrint("\n");
#endif
    
    /* Determine U/D gesture */
    if( pApds->gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
        pApds->gesture_ud_count_ = 1;
    } else if( pApds->gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        pApds->gesture_ud_count_ = -1;
    } else {
        pApds->gesture_ud_count_ = 0;
    }
    
    /* Determine L/R gesture */
    if( pApds->gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
        pApds->gesture_lr_count_ = 1;
    } else if( pApds->gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        pApds->gesture_lr_count_ = -1;
    } else {
        pApds->gesture_lr_count_ = 0;
    }
    
    /* Determine Near/Far gesture */
    if( (pApds->gesture_ud_count_ == 0) && (pApds->gesture_lr_count_ == 0) ) {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
            
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                pApds->gesture_near_count_++;
            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
                pApds->gesture_far_count_++;
            }
            
            if( (pApds->gesture_near_count_ >= 10) && (pApds->gesture_far_count_ >= 2) ) {
                if( (ud_delta == 0) && (lr_delta == 0) ) {
                    pApds->gesture_state_ = NEAR_STATE;
                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
                    pApds->gesture_state_ = FAR_STATE;
                }
                return TRUE;
            }
        }
    } else {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
                
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                pApds->gesture_near_count_++;
            }
            
            if( pApds->gesture_near_count_ >= 10 ) {
                pApds->gesture_ud_count_ = 0;
                pApds->gesture_lr_count_ = 0;
                pApds->gesture_ud_delta_ = 0;
                pApds->gesture_lr_delta_ = 0;
            }
        }
    }
    
#if DEBUG
    MyPrint("UD_CT: ");
    MyPrint(pApds->gesture_ud_count_);
    MyPrint(" LR_CT: ");
    MyPrint(pApds->gesture_lr_count_);
    MyPrint(" NEAR_CT: ");
    MyPrint(pApds->gesture_near_count_);
    MyPrint(" FAR_CT: ");
    MyPrint(pApds->gesture_far_count_);
    MyPrint("\n");
    MyPrint("----------");
    MyPrint("\n");
#endif
    
    return FALSE;
}

/**
 * @brief Determines swipe direction or near/far state
 *
 * @return TRUE if near/far event. FALSE otherwise.
 */
BOOL decodeGesture(APDS9960_t *pApds)
{
    /* Return if near or far event is detected */
    if( pApds->gesture_state_ == NEAR_STATE ) {
        pApds->gesture_motion_ = DIR_NEAR;
        return TRUE;
    } else if ( pApds->gesture_state_ == FAR_STATE ) {
        pApds->gesture_motion_ = DIR_FAR;
        return TRUE;
    }
    
    /* Determine swipe direction */
    if( (pApds->gesture_ud_count_ == -1) && (pApds->gesture_lr_count_ == 0) ) {
        pApds->gesture_motion_ = DIR_UP;
    } else if( (pApds->gesture_ud_count_ == 1) && (pApds->gesture_lr_count_ == 0) ) {
        pApds->gesture_motion_ = DIR_DOWN;
    } else if( (pApds->gesture_ud_count_ == 0) && (pApds->gesture_lr_count_ == 1) ) {
        pApds->gesture_motion_ = DIR_RIGHT;
    } else if( (pApds->gesture_ud_count_ == 0) && (pApds->gesture_lr_count_ == -1) ) {
        pApds->gesture_motion_ = DIR_LEFT;
    } else if( (pApds->gesture_ud_count_ == -1) && (pApds->gesture_lr_count_ == 1) ) {
        if( abs(pApds->gesture_ud_delta_) > abs(pApds->gesture_lr_delta_) ) {
            pApds->gesture_motion_ = DIR_UP;
        } else {
            pApds->gesture_motion_ = DIR_RIGHT;
        }
    } else if( (pApds->gesture_ud_count_ == 1) && (pApds->gesture_lr_count_ == -1) ) {
        if( abs(pApds->gesture_ud_delta_) > abs(pApds->gesture_lr_delta_) ) {
            pApds->gesture_motion_ = DIR_DOWN;
        } else {
            pApds->gesture_motion_ = DIR_LEFT;
        }
    } else if( (pApds->gesture_ud_count_ == -1) && (pApds->gesture_lr_count_ == -1) ) {
        if( abs(pApds->gesture_ud_delta_) > abs(pApds->gesture_lr_delta_) ) {
            pApds->gesture_motion_ = DIR_UP;
        } else {
            pApds->gesture_motion_ = DIR_LEFT;
        }
    } else if( (pApds->gesture_ud_count_ == 1) && (pApds->gesture_lr_count_ == 1) ) {
        if( abs(pApds->gesture_ud_delta_) > abs(pApds->gesture_lr_delta_) ) {
            pApds->gesture_motion_ = DIR_DOWN;
        } else {
            pApds->gesture_motion_ = DIR_RIGHT;
        }
    } else {
        return FALSE;
    }
    
    return TRUE;
}

UINT8 convertByteToCharacter(UINT8 byte, UINT8* buf, UINT8 max){
  
  UINT8 len = 0;
  UINT8 temp = byte;
  UINT8 idx = max - 1;
  if(byte == 0){
     len = 1;
    *(buf + idx) = 0x30;
    return len;
  }
   
  while( ( temp/10 != 0 || (temp/10 == 0 && temp % 10 != 0)) && len <= max ){
    *(buf + idx) = (0x30 + temp % 10);
    temp /= 10;
    len++;
    idx--;
  }
  return len;
}