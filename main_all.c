#include <plib.h>
#include <stdlib.h>
#include <string.h>
#include "ADS1115.h"
#include "MPU6050.h"
#include "APDS9960.h"
#include "SSD1306.h"
#include "Config.h"
#include "serialParser.h"

#if defined (__32MX150F128B__)
// Configuration Bit settings
// SYSCLK = 32 MHz (8MHz Crystal / FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 32 MHz (SYSCLK / FPBDIV)
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF (Initial)
// Other options are don't care
#pragma config FPLLMUL = MUL_16, FPLLIDIV = DIV_2, FPLLODIV = DIV_2, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#pragma config WDTPS = PS4
#define SYS_FREQ (32000000L)
#endif

#define	GetPeripheralClock()		(SYS_FREQ/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(SYS_FREQ)

// Define setup parameters for OpenADC10 function
// Turn module on | Ouput in integer format | Trigger mode auto | Enable autosample
#define config1     ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
// ADC ref external | Disable offset test | Disable scan mode | Perform 2 samples | Use dual buffers | Use alternate mode
#define config2     ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF //| ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_ON
// Use ADC internal clock | Set sample time
#define config3     ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15
// Do not assign channels to scan
#define configscan  SKIP_SCAN_ALL

// ADC
#define configport  ENABLE_AN3_ANA

// UART
#define UART_MODULE_ID UART2
#define RX_BUFF_SIZE    128

// I2S Microphone Sample Rate = 10kHz
#define I2S_SAMPLE_RATE 10000

// I2C
#define SLEEP_INTERVAL              16
#define ACC_HZ                      25
#define ECG_HZ                      250
#define ONEWIRE_HZ                  1
#define I2C_CLOCK_FREQ              500000
#define ACC_SAMPLE_RATE             10
#define I2C_BUS                     I2C1
#define ADS1115_ADDRESS             ADS1115_ADDRESS_ADDR_GND
#define MPU6050_ADDRESS             0b1101000
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_PWR1_SLEEP_BIT      6

typedef struct _ACCDATA {
    INT16 ax;
    INT16 ay;
    INT16 az;
    INT16 gx;
    INT16 gy;
    INT16 gz;
}ACCDATA;

typedef enum _ACCSTATE {
    ACC_IDLE = 0,
    ACC_CHANGE
}ACCSTATE;

typedef enum _POSTURE_STATE {
    POST_LYING = 0,
    POST_STANDING
}POSTURE_STATE;

// Function Prototypes
void DelayMs(UINT);
BOOL InitialUART();
BOOL InitialI2C();
BOOL InitialACC();
INT16 ReadADC();
INT16 ReadTMP();
struct _ACCDATA ReadACC();
void SendDataBuffer(const BYTE *buffer, UINT32 size);
void MyPrintUint8(UINT8 val);
void MyPrintInt(INT val);
void MyPrint(const BYTE *);
BOOL StartTransfer(BOOL restart);
BOOL TransmitOneByte(UINT8 data);
void StopTransfer(void);
void SendADCWord(BYTE regAddr, UINT16 data);
UINT16 ReadTMPWord(BYTE regAddr);
BOOL ACCConnected(BYTE regAddr);
void SendACCByte(BYTE regAddr, UINT8 data);
void ReadACCWord(BYTE regAddr, BYTE *acc_data);

I2C_7_BIT_ADDRESS   SlaveAddress;

enum _POSTURE_STATE _posture_state;
const UINT8 TEMP_THEME = 0;
const UINT8 HR_THEME = 1;
UINT8 displayTheme;
UINT8 maxTheme = 2;
BOOL themeChange = FALSE;
UINT16 countForRefresh = 0;
BOOL waitForRefresh = FALSE;

// Serial communication
const BYTE PREFIX[2] = {0xFF, 0x7F};
const BYTE INIT_UART[2] = {0x00, 0x00};

const BYTE TYPE1 = 0x01;      //ECG
const BYTE TYPE2 = 0x02;      //ACC
const BYTE TYPE3 = 0x03;      //TEMP

const BYTE LYING = 0x00;      //Lying
const BYTE STANDING = 0x01;   //Standing
const BYTE STEP = 0x02;       //Step

RingBuffer ringbuffer;

int main() {
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above.
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    // Explorer-16 LEDs are on lower 8-bits of PORTA and to use all LEDs, JTAG port must be disabled.
    mJTAGPortEnable(DEBUG_JTAGPORT_OFF);

    PPSInput(2, U2RX, RPB5);  // UART2 Rx
    PPSOutput(4,RPB10, U2TX); // UART2 Tx

    // Ensure the ADC is off before setting the configuration
    CloseADC10();

    // Use ground as neg ref for A | use AN4 for input A
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN3);

    // Configure ADC using the parameters defined above
    OpenADC10( config1, config2, config3, configport, configscan);

    DisableWDT();

    int ac = 0;

    InitialUART();
    InitialI2C();

    if(ACCConnected(MPU6050_RA_GYRO_CONFIG)){
        InitialACC();
        ac =1;
    }
    DelayMs(100);

    //  ---INITIALIZED---

    /*ECG detection variables*/
    UINT  count = 0;
    INT16 ecgValue = 0;
    INT16 ecgRawValue = 0;
    INT16 ecgAverage = 0;
    INT16 diff1;
    INT16 diff2;
    INT16 difftemp;
    INT16 interval = 0;
    INT   ecgAccumulator = 0;
    INT16 ecgThreshold = 40;
    BOOL  isHoldRef = FALSE;
    BOOL  isHoldCandid = FALSE;
    BOOL  isRising = FALSE;
    INT16 beginFlag = 0;
    INT16 candidFlag = 0;
    INT16 beatRate = 120;
    INT16 pastBeatRate = -1;

    INT16 ecgCount = 0;
    INT16 ecgWindow = 8;
    INT16 ecgDelay = 20;
    INT16 ecgRaw[8];
    INT   ecgRawSum = 0;
    INT16 ecgFilteredData[160];   // ecgWindow * ecgDelay
    UINT  ecgTempMax = 0;
    /*End of declaration.*/

    /*Pedometer variables*/
    enum _ACCSTATE _accstate;
    UINT  accCount = 0;
    INT16 accWindow = 6;
    INT16 accDelay = 4;
    INT   accRaw[accWindow];
    INT   accRawSum = 0;
    INT   accFilteredData[24];    // accWindow * accDelay
    INT16 accTolerant = 0;
    INT   accMax = 0;
    INT   accMin = 0;
    INT   accThreshold = 0;
    INT   accValue;
    BOOL  isPositive = FALSE;
    /*End of declaration*/

    /*Temperature variables*/
    INT16 pastTmp;
    INT16 tmpRaw;
    float correctedTemp;
    UINT8 tmpInteger;
    UINT8 tmpDecimal;
    UINT8 tmpPastDecimal;
    /*End of declaration*/
    
    /*Calculate sampling rate*/
    INT16 sampleFreq = 235;
    
    /*End of declaration*/

    /*Gesture detection variables*/
    APDS9960_t apds;
    UINT DelayCounter = 8;
    UINT8 loopCounter = 0;
    UINT gestureState = DIR_NONE;
    /*End of declaration*/

    MyPrint("Initialize APDS9960 ....\n");
    if(initAPDS9960(&apds) == TRUE){
        MyPrint("Initialize APDS9960 successfully!.\n");
    } else {
        MyPrint("Failed to initialize APDS9960.\n");
    }
    if(enableGestureSensor(&apds, FALSE)){
        MyPrint("Gesture sensor is now running.\n");
    } else {
        MyPrint("Something went wrong during gesture sensor init!\n");
    }
    
    initSSD1306(1);
    beginSSD1306(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, -1);
    stopscroll();
    display();
    DelayMs(2000);
    
    displayTemperature(3, 6, 5);
    DelayMs(2000);
    
    displayBeatRatePerMin(1, 2, 7);
    DelayMs(2000);

    RingBuffer_init(&ringbuffer);
    
    UINT i = 0; // Iteration counter
    while(i < accWindow){
        accRaw[i] = 0;
        i++;
    }
    i = 0;
    while(i < accDelay*accWindow){
        accFilteredData[i] = 0;
        i++;
    }
    i = 0;
    while(i < ecgWindow){
        ecgRaw[i] = 0;
        i++;
    }
    i = 0;
    while(i < ecgDelay*ecgWindow){
        ecgFilteredData[i] = 0;
        i++;
    }

    EnableADC10();
    //EnableWDT();
    
    displayTheme = TEMP_THEME;
    MyPrint("End of initialization.\n");
    

    //  ---Main LOOP---
    while(1){

        ClearWDT();
        count++;

        if(waitForRefresh != TRUE){
            /*ECG START*/
            if(displayTheme == HR_THEME){
                if(pastBeatRate != beatRate || themeChange == TRUE){
                    displayBeatRatePerMin(beatRate/100, (beatRate%100)/10, beatRate%10);
                    pastBeatRate = beatRate;
                    themeChange = FALSE;
                }
            }
            if(displayTheme == TEMP_THEME){
                if((tmpRaw != pastTmp) || themeChange == TRUE){
                    correctedTemp = tmpRaw*0.0625;
                
                    tmpInteger = (UINT8)correctedTemp;
                    tmpDecimal = (UINT8)(correctedTemp * 10 - tmpInteger * 10);
                
                    if(tmpPastDecimal != tmpDecimal || themeChange == TRUE){
                        displayTemperature(tmpInteger/10, tmpInteger%10, tmpDecimal);
                    }
                    tmpPastDecimal = tmpDecimal;
                    themeChange = FALSE;
                }
            }
        }
        else{
            if(countForRefresh < 240)
                countForRefresh++;
            else{
                countForRefresh = 0;
                waitForRefresh = FALSE;
            }
        }
        
//        if(1) {
//
//            ecgRawValue = ReadADC();
//            ecgRawSum -= ecgRaw[ecgCount%ecgWindow];
//            ecgRaw[ecgCount%ecgWindow] = ecgRawValue;
//            ecgRawSum += ecgRaw[ecgCount%ecgWindow];
//            ecgCount++;
//            ecgCount %= 160;
//            ecgFilteredData[ecgCount] = ecgRawSum/ecgWindow;
//            ecgValue = ecgFilteredData[ecgCount];
//
//            ecgAccumulator += ecgValue;
//            if(count % ecgWindow == 0){     // Frequency of updating average value could be tuned.
//                ecgAverage = ecgAccumulator/(ecgWindow);
//                ecgAccumulator = 0;
//
//                if(count % 160 == 0){
//                    ecgThreshold = max(ecgThreshold, 0.5*abs(ecgTempMax-ecgAverage));
//                }
//            }
//
//
//            if( abs(ecgValue-ecgAverage) > ecgThreshold ){
//                if( isHoldRef == FALSE ){
//                    beginFlag = count;
//                    diff1 = ecgValue-ecgAverage;
//                    isHoldRef = TRUE;
//                    ecgTempMax = ecgValue; 
//                }
//                else{
//                    interval = count-beginFlag;
//                    difftemp = ecgValue-ecgAverage;
//                    if(interval >= 300){
//                        isHoldRef = FALSE;
//                        isHoldCandid = FALSE;
//                        ecgThreshold = 40;
//                        ecgTempMax = ecgAverage + 10; 
//                    }
//                    else if(interval == 1){
//                        if(difftemp*diff1 > 0 && abs(difftemp) > abs(diff1)){
//                            beginFlag = count;
//                            diff1 = difftemp;
//                            ecgTempMax = ecgValue;
//                        }
//                    }
//                    else if(interval < 300 && interval > 80 && difftemp*diff1 > 0){
//                        if(isHoldCandid == FALSE){
//                            diff2 = difftemp;
//                            candidFlag = count;
//                            isHoldCandid = TRUE;
//                        }
//                        else{
//                            if(abs(difftemp) > abs(diff2)){
//                                diff2 = difftemp;
//                                candidFlag = count;
//                                isRising = TRUE;
//                            }
//                            else if(isRising == TRUE){
//                                INT16 deltaN = candidFlag-beginFlag;
//                                beatRate = sampleFreq*60/deltaN ;
//
//                                SendDataBuffer(&INIT_UART, 1);
//                                SendDataBuffer(&PREFIX, 2);
//                                SendDataBuffer(&TYPE1, 1);
//                                SendDataBuffer(&deltaN, 1);
//
//                                beginFlag = candidFlag;
//                                diff1 = diff2;
//                                isHoldCandid = FALSE;
//                                isRising = FALSE;
//                                ecgTempMax = ecgValue;
//                            }
//                            else{
//                                isHoldCandid = FALSE;
//                                isRising = FALSE;
//                                ecgThreshold = 40;
//                                ecgTempMax = ecgAverage + 10;
//                            }
//                        }
//                    }
//                }
//            }
//        }
        /*ECG END*/

        /*ACC START*/
//        if(count % 10 == 0){
//            if(ACCConnected(MPU6050_RA_ACCEL_XOUT_H)){
//
//                InitialI2C();
//                InitialACC();
//                ac=1;
//                struct _ACCDATA accdata = ReadACC();
//
//                accRawSum -= accRaw[accCount%accWindow];
//                accRaw[accCount%accWindow] = abs(accdata.ax) + abs(accdata.ay) + abs(accdata.az);
//                accRawSum += accRaw[accCount%accWindow];
//                accCount++;
//                accCount %= (accDelay*accWindow);
//                accFilteredData[accCount] = accRawSum/accWindow;
//                accValue = accFilteredData[accCount];
//    
//                if(accCount == 0){
//                    i = 0;
//                    accMax = accFilteredData[i];
//                    accMin = accFilteredData[i];
//                    while(i < accDelay*accWindow){
//                        if(accFilteredData[i] > accMax)
//                            accMax = accFilteredData[i];
//                        else{
//                            if(accFilteredData[i] < accMin)
//                                accMin = accFilteredData[i];
//                        }
//                        i++;
//                    }
//                    accThreshold = (accMax + accMin)/2;
//                    if(abs(accMax-accMin) < 3500){
//                        _accstate = ACC_IDLE;
//                        if( abs(accdata.az) > 10000 ){
//                            //myPrint("Lying\n");
//                            if(_posture_state != POST_LYING) {
//                                _posture_state = POST_LYING;
//                                SendDataBuffer(&PREFIX, 2);
//                                SendDataBuffer(&TYPE2, 1);
//                                SendDataBuffer(&LYING, 1);
//                            }
//                        }
//                        else{
//                            //myPrint("Standing\n");
//                            if(_posture_state != POST_STANDING) {
//                                _posture_state = POST_STANDING;
//                                SendDataBuffer(&PREFIX, 2);
//                                SendDataBuffer(&TYPE2, 1);
//                                SendDataBuffer(&STANDING, 1);
//                            }
//                        }
//                    }
//                    else
//                        _accstate = ACC_CHANGE;
//                }
//                if( _accstate != ACC_IDLE){
//                    if(accValue > accThreshold){
//                        if(isPositive == FALSE){
//                            isPositive = TRUE;
//                            accTolerant = 0;
//                        }
//                        else
//                            accTolerant++;
//                    }
//                    else{
//                        if(isPositive == TRUE && accTolerant >= 3){
//                            //myPrint("Step!\n");
//                            SendDataBuffer(&PREFIX, 2);
//                            SendDataBuffer(&TYPE2, 1);
//                            SendDataBuffer(&STEP, 1);
//                        }
//                        isPositive = FALSE;
//                    }
//                }
//            }
//            else{
////                myPrint("(ACC) DISCONNECTED!!\n");
//                ac=0;
//            }
//        }
        /*ACC END*/

        /*TEMP START*/
        if(count % 235==0){
            pastTmp = tmpRaw;
            tmpRaw = ReadTMP();
        }
        /*TEMP END*/
        
        /*GESTURE START*/
        UINT result = handleGesture(&apds, loopCounter);


        
        /*GESTURE END*/
        
        //PowerSaveSleep();
        DelayMs(4);
    }

    return (EXIT_SUCCESS);
}

BOOL InitialUART()
{
    // Enable UART
    UARTConfigure(UART_MODULE_ID, UART_ENABLE_PINS_TX_RX_ONLY | UART_ENABLE_HIGH_SPEED);
    UARTSetFifoMode(UART_MODULE_ID, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART_MODULE_ID, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART_MODULE_ID, GetPeripheralClock(), 115200);
    UARTEnable(UART_MODULE_ID, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    
//    // Configure UART RX Interrupt
//    INTEnable(INT_SOURCE_UART_RX(UART_MODULE_ID), INT_ENABLED);
//    INTSetVectorPriority(INT_VECTOR_UART(UART_MODULE_ID), INT_PRIORITY_LEVEL_2);
//    INTSetVectorSubPriority(INT_VECTOR_UART(UART_MODULE_ID), INT_SUB_PRIORITY_LEVEL_0);
//    INTEnableInterrupts();
//
//    // Enable multi-vector interrupts
//    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
//    INTEnableInterrupts();
    
    return TRUE;
}

BOOL InitialI2C()
{

    // Enable I2C
    int actualClock = I2CSetFrequency(I2C_BUS, GetPeripheralClock(), I2C_CLOCK_FREQ);
    //myPrintInt(actualClock);
    if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
    {
        MyPrint("Error: I2C1 clock frequency error exceeds 10%.\n");
    }
    // Enable the I2C bus
    I2CEnable(I2C_BUS, TRUE);

    return TRUE;
}

BOOL InitialACC()
{

    // ACC initialization
    UINT8 MPU_CONFIG = 0X00;
    SendACCByte(MPU6050_RA_GYRO_CONFIG, MPU_CONFIG);
    SendACCByte(MPU6050_RA_ACCEL_CONFIG, MPU_CONFIG);
    UINT8 MPU_PWR = 0X01;
    SendACCByte(MPU6050_RA_PWR_MGMT_1, MPU_PWR);

    return TRUE;
}

INT16 ReadADC()
{
    INT16 value = ReadADC10(0);
    mAD1ClearIntFlag();
    return value;
}

struct _ACCDATA ReadACC()
{
    BYTE ACC_DATA[14];
    //INT16 ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    struct _ACCDATA accdata;
    ReadACCWord(MPU6050_RA_ACCEL_XOUT_H, ACC_DATA);

    accdata.ax = (ACC_DATA[0] << 8) + ACC_DATA[1];
    accdata.ay = (ACC_DATA[2] << 8) + ACC_DATA[3];
    accdata.az = (ACC_DATA[4] << 8) + ACC_DATA[5];
    accdata.gx = (ACC_DATA[8] << 8) + ACC_DATA[9];
    accdata.gy = (ACC_DATA[10] << 8) + ACC_DATA[11];
    accdata.gz = (ACC_DATA[12] << 8) + ACC_DATA[13];

    return accdata;

}

INT16 ReadTMP()
{
    INT16 data = ReadTMPWord(ADS1115_RA_CONVERSION);
    if(data == 0xFFFF)
        return data;

//    SendDataBuffer(&PREFIX, 2);
//    SendDataBuffer(&TYPE3, 1);
//    SendDataBuffer(&data, 2);
    return data;
}

void DelayMs(UINT msec)
{
    UINT tWait, tStart;

    tWait=(GetInstructionClock()/2000)*msec;        //
    tStart=ReadCoreTimer();
    while((ReadCoreTimer()-tStart)<tWait);        // wait for the time to pass
}

void MyPrintUint8(UINT8 val)
{
    val = val & 0xFF;
    MyPrint(val);
}

void MyPrintInt(INT val)
{
    BYTE buffer[40];
    itoa(buffer, val, 10);
    MyPrint(buffer);
}

void MyPrint(const BYTE *BYTEArray)
{
    SendDataBuffer(BYTEArray, strlen(BYTEArray));
}


void SendDataBuffer(const BYTE *buffer, UINT32 size)
{
    while(size)
    {
        while(!UARTTransmitterIsReady(UART_MODULE_ID))
            ;

        UARTSendDataByte(UART_MODULE_ID, *buffer);

        buffer++;
        size--;
    }

    while(!UARTTransmissionHasCompleted(UART_MODULE_ID))
        ;
}



/*******************************************************************************
 Function:
 BOOL StartTransfer( BOOL restart )

 Summary:
 Starts (or restarts) a transfer to/from the EEPROM.

 Description:
 This routine starts (or restarts) a transfer to/from the EEPROM, waiting (in
 a blocking loop) until the start (or re-start) condition has completed.

 Precondition:
 The I2C module must have been initialized.

 Parameters:
 restart - If FALSE, send a "Start" condition
 - If TRUE, send a "Restart" condition

 Returns:
 TRUE    - If successful
 FALSE   - If a collision occured during Start signaling

 Example:
 <code>
 StartTransfer(FALSE);
 </code>

 Remarks:
 This is a blocking routine that waits for the bus to be idle and the Start
 (or Restart) signal to complete.
 *****************************************************************************/

BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;
    int c;
    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(I2C_BUS);
    }
    else
    {
        //for(c=0; c<10000; c++);
        // Wait for the bus to be idle, then start the transfer
        //myPrint("bidle\n");
        while( !I2CBusIsIdle(I2C_BUS) );
        //myPrint("aidle\n");
        if(I2CStart(I2C_BUS) != I2C_SUCCESS)
        {
            MyPrint("Error: Bus collision during transfer Start\n");

            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(I2C_BUS);

    } while ( !(status & I2C_START) );
    return TRUE;
}


/*******************************************************************************
 Function:
 BOOL TransmitOneByte( UINT8 data )

 Summary:
 This transmits one byte to the EEPROM.

 Description:
 This transmits one byte to the EEPROM, and reports errors for any bus
 collisions.

 Precondition:
 The transfer must have been previously started.

 Parameters:
 data    - Data byte to transmit

 Returns:
 TRUE    - Data was sent successfully
 FALSE   - A bus collision occured

 Example:
 <code>
 TransmitOneByte(0xAA);
 </code>

 Remarks:
 This is a blocking routine that waits for the transmission to complete.
 *****************************************************************************/

BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(I2C_BUS));

    // Transmit the byte
    if(I2CSendByte(I2C_BUS, data) == I2C_MASTER_BUS_COLLISION)
    {
        MyPrint("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(I2C_BUS));

    return TRUE;
}


/*******************************************************************************
 Function:
 void StopTransfer( void )

 Summary:
 Stops a transfer to/from the EEPROM.

 Description:
 This routine Stops a transfer to/from the EEPROM, waiting (in a
 blocking loop) until the Stop condition has completed.

 Precondition:
 The I2C module must have been initialized & a transfer started.

 Parameters:
 None.

 Returns:
 None.

 Example:
 <code>
 StopTransfer();
 </code>

 Remarks:
 This is a blocking routine that waits for the Stop signal to complete.
 *****************************************************************************/

void StopTransfer( void )
{
    I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(I2C_BUS);

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(I2C_BUS);

    } while ( !(status & I2C_STOP) );
}

void SendADCWord(BYTE regAddr, UINT16 data)
{

    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, ADS1115_ADDRESS, I2C_WRITE);
    UINT8 i2cData[4];
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = regAddr;             // register address
    i2cData[2] = data >> 8;           // Data 1st byte
    i2cData[3] = data;                // Data 2nd byte

    // Start the transfer to write data to the EEPROM
    if( !StartTransfer(FALSE) )
    {
        MyPrint("Can Not Start Transfer to Write Data to EEPROM\n");
    }

    // Transmit all data
    int Index = 0;
    BOOL Success = TRUE;
    while( Success && (Index < 4) )
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
        MyPrint("End transfer error.\n");
    }


    // Wait for EEPROM to complete write process, by polling the ack status.
    BOOL Acknowledged = FALSE;
    do
    {
        // Start the transfer to address the EEPROM
        if( !StartTransfer(FALSE) )
        {
            MyPrint("Address Transfer Error\n");
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

}

void SendACCByte(BYTE regAddr, UINT8 data)
{

    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, MPU6050_ADDRESS, I2C_WRITE);
    UINT8 i2cData[3];
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = regAddr;             // register address
    i2cData[2] = data;           // Data byte

    // Start the transfer to write data to the EEPROM
    if( !StartTransfer(FALSE) )
    {
        MyPrint("Start Transfer Error.\n");
    }

    // Transmit all data
    int Index = 0;
    BOOL Success = TRUE;
    while( Success && (Index < 3) )
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


    // Wait for EEPROM to complete write process, by polling the ack status.
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

}

BOOL ACCConnected(BYTE regAddr)
{
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, MPU6050_ADDRESS, I2C_WRITE);
    UINT8 i2cData[2];
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = regAddr;              // register address

    // Start the transfer to read the EEPROM.
    if( !StartTransfer(FALSE) )
    {
        MyPrint("Start Transfer Error.\n");
    }
    // Address the EEPROM.
    int Index = 0;
    BOOL Success = TRUE;
    while( Success && (Index < 2) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;
        }
        else
        {
            Success = FALSE;
        }

        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(I2C_BUS))
        {
            MyPrint("(ACC) Disconnected.\n");
            MyPrint("Error: Sent byte was not acknowledged (ReadI2CWord)\n");
            Success = FALSE;
        }
    }
    StopTransfer();
    return Success;
}

void ReadACCWord(BYTE regAddr, BYTE *acc_data)

{
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, MPU6050_ADDRESS, I2C_WRITE);
    UINT8 i2cData[2];
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = regAddr;              // register address

    // Start the transfer to read the EEPROM.
    if( !StartTransfer(FALSE) )
    {
        MyPrint("Start Transfer Error.\n");
    }
    // Address the EEPROM.
    int Index = 0;
    BOOL Success = TRUE;
    while( Success && (Index < 2) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;
        }
        else
        {
            Success = FALSE;
        }

        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(I2C_BUS))
        {
            MyPrint("(ACC) Disconnected.\n");
            MyPrint("Error: Sent byte was not acknowledged (ReadI2CWord)\n");
            Success = FALSE;
        }
    }
    // Restart and send the EEPROM's internal address to switch to a read transfer
    if(Success)
    {
        // Send a Repeated Started condition
        if( !StartTransfer(TRUE) )
        {
            MyPrint("Start Transfer Error.\n");
        }

        // Transmit the address with the READ bit set
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, MPU6050_ADDRESS, I2C_READ);
        if (TransmitOneByte(SlaveAddress.byte))
        {
            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(I2C_BUS))
            {
                MyPrint("Error: Sent byte was not acknowledged (ReadI2CWord)\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }
    //myPrint("here\n");
    // Read the data from the desired address
    //UINT16 data = 0;
    if(Success)
    {
        if(I2CReceiverEnable(I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
        {
            MyPrint("Error: I2C Receive Overflow\n");
            Success = FALSE;
        }
        else
        {
            //myPrint("ininder\n");
            int i;
            while(!I2CReceivedDataIsAvailable(I2C_BUS));
            I2CAcknowledgeByte(I2C_BUS, TRUE);
            acc_data[0] = I2CGetByte(I2C_BUS);
            while(!I2CAcknowledgeHasCompleted(I2C_BUS));

            for(i=1; i<13; i++){
                I2CReceiverEnable(I2C_BUS, TRUE);
                while(!I2CReceivedDataIsAvailable(I2C_BUS));
                I2CAcknowledgeByte(I2C_BUS, TRUE);
                acc_data[i] = I2CGetByte(I2C_BUS);
                while(!I2CAcknowledgeHasCompleted(I2C_BUS));
            }

            I2CReceiverEnable(I2C_BUS, TRUE);
            while(!I2CReceivedDataIsAvailable(I2C_BUS));
            acc_data[13] = I2CGetByte(I2C_BUS);
            //data = (byte1 << 8) + byte2;
        }

    }
    // End the transfer (stop here if an error occured)
    // myPrint("321\n");
    StopTransfer();
    //myPrint("123\n");
    if(!Success)
    {
        MyPrint("End Transfer Error.\n");
    }
    //myPrint("aftertransmit\n");
    //return acc_data;
}

UINT16 ReadTMPWord(BYTE regAddr)
{
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, 0x49, I2C_WRITE);
    UINT8 i2cData[2];
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = regAddr;              // register address

    // Start the transfer to read the EEPROM.
    if( !StartTransfer(FALSE) )
    {
        //        myPrint("Start Transfer Error.\n");
    }
    // Address the EEPROM.
    int Index = 0;
    BOOL Success = TRUE;
    while( Success & (Index < 2) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;
        }
        else
        {
            //            myPrint("TransferOneByte Error.\n");
            Success = FALSE;
        }

        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(I2C_BUS))
        {
            //            myPrint("(TMP) Disconnected.\n");
            //            myPrint("Error: Sent byte was not acknowledged\n");
            Success = FALSE;
        }
    }
    // Restart and send the EEPROM's internal address to switch to a read transfer
    if(Success)
    {
        // Send a Repeated Started condition
        if( !StartTransfer(TRUE) )
        {
            //            myPrint("Start Transfer Error.\n");
        }

        // Transmit the address with the READ bit set
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, 0x49, I2C_READ);
        if (TransmitOneByte(SlaveAddress.byte))
        {
            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(I2C_BUS))
            {
                //                myPrint("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            //            myPrint("TransferOneByte Error.\n");
            Success = FALSE;
        }
    }

    // Read the data from the desired address
    UINT16 data = 0;
    if(Success)
    {
        //myPrint("In data read!\n");
        if(I2CReceiverEnable(I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
        {
            //            myPrint("Error: I2C Receive Overflow\n");
            Success = FALSE;
        }
        else
        {
            while(!I2CReceivedDataIsAvailable(I2C_BUS));
            I2CAcknowledgeByte(I2C_BUS, TRUE);
            BYTE byte1 = I2CGetByte(I2C_BUS);
            while(!I2CAcknowledgeHasCompleted(I2C_BUS));

            I2CReceiverEnable(I2C_BUS, TRUE);
            while(!I2CReceivedDataIsAvailable(I2C_BUS));
            I2CAcknowledgeByte(I2C_BUS, TRUE);
            BYTE byte2 = I2CGetByte(I2C_BUS);
            while(!I2CAcknowledgeHasCompleted(I2C_BUS));

            data = (byte1 << 4) | (byte2>>4);
        }

    }

    // End the transfer (stop here if an error occured)
    StopTransfer();
    if(!Success)
    {
        //        myPrint("End Transfer Error.\n");
        return 0xFFFF;
    }
    return data;
}

// UART 2 interrupt handler, set at priority level 2
void __ISR(_UART2_VECTOR, IPL2SOFT) IntUart2Handler(void)
{
	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART_MODULE_ID)))
	{   
        while(!UARTReceivedDataIsAvailable(UART_MODULE_ID));
        RingBuffer_write(&ringbuffer, UARTGetDataByte(UART_MODULE_ID), 1);
        
        if(RingBuffer_available_data(&ringbuffer) >= 4){
            
        }
        
        // Clear the RX interrupt Flag
	    INTClearFlag(INT_SOURCE_UART_RX(UART_MODULE_ID));
	}

	// We don't care about TX interrupt
	if ( INTGetFlag(INT_SOURCE_UART_TX(UART_MODULE_ID)) )
	{
        INTClearFlag(INT_SOURCE_UART_TX(UART_MODULE_ID));
	}

    // We don't care about Error interrupt
	if ( INTGetFlag(INT_SOURCE_UART_ERROR(UART_MODULE_ID)) )
	{
        INTClearFlag(INT_SOURCE_UART_ERROR(UART_MODULE_ID));
	}
}