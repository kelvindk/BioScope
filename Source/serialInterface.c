#include "hal_uart.h"
#include "serialInterface.h"
#include "BioScope.h"
#include "bcomdef.h"
#include "hal_led.h"
#include "hal_sensor.h"
#include "hal_i2c.h"

#include "string.h"
#include "gatt.h"
#include "simpleGATTprofile.h"

//local function
static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg );

uint8 serialInterface_TaskID;           // Task ID for internal task/event processing

uint8 pktBuf[RX_BUFF_SIZE];
uint8 preamble = FALSE; // preamble includes FF, 7F, Type
uint8 pktLength = 0; // target packet length
uint8 pktRxByteOffset = 0;              // current received bytes offset
uint8 numBytes;
uint8 RxByte;

uint8 cameraAddr = (CAM_ADDR << 5);  	// addr
uint16 picTotalLen = 0;            	// picture length
uint16 pktCnt = 0;
uint16 tmpPktIdx = 0;
uint16 lastPktLen = 0;
uint8 isLastPkt = 0;
uint16 seqNum = 0;
uint8 waitBLEAck = 0;
uint8 blePktOffset = 0;

uint16 retransmitSize = 0;
uint16 tmpRetransmitIdx = 0;
uint16 retransmitBuf[18];

void SerialInterface_Init( uint8 task_id )
{
  serialInterface_TaskID = task_id;
  //NPI_InitTransport(cSerialPacketParser);
}

uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    
    if ( (pMsg = osal_msg_receive( serialInterface_TaskID )) != NULL )
    {
      SerialInterface_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  // Discard unknown events
  return 0;
}

static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  default:
    // do nothing
    break;
  }
}

void cSerialPacketParser( uint8 port, uint8 events )
{
  if((preamble) ) {
    if(pktRxByteOffset != 0) {
      numBytes = NPI_RxBufLen();
      if(numBytes < pktRxByteOffset) {
        (void)NPI_ReadTransport(pktBuf+(pktLength-pktRxByteOffset+1), numBytes);
        pktRxByteOffset -= numBytes;
        return;
      }
      else {
        (void)NPI_ReadTransport(pktBuf+(pktLength-pktRxByteOffset+1), pktRxByteOffset);
        pktRxByteOffset = 0;
      }
    }
    if(pktRxByteOffset == 0) {
      // got sufficient bytes in a packet
//      serialPacketHandler();
      if( gapProfileState == GAPROLE_CONNECTED )
      {
        HalLedSet( HAL_LED_2, HAL_LED_MODE_TOGGLE );
        (void)sendNotification((uint8 *)&pktBuf, pktLength+1);
      }
      preamble = FALSE;
      return;
    }
  }
  
  numBytes = NPI_RxBufLen();
   
  if(numBytes < 3)
    return;
   
  (void)NPI_ReadTransport((uint8 *)&RxByte, 1);
  if(RxByte != 0xFF)
    return;
  else{
    (void)NPI_ReadTransport((uint8 *)&RxByte, 1);
    if(RxByte != 0x7F)
      return;
    else { // Then handle the packet depending on type
              
      (void)NPI_ReadTransport((uint8 *)&pktBuf, 1);
      preamble = TRUE;
      switch(pktBuf[0]) {
        case 1: //ECG beat
          pktRxByteOffset = 1;
          break;
        case 2: //ACC
          pktRxByteOffset = 1;
          break;
        case 3: //Temp
          pktRxByteOffset = 2;
          break;
        default:
          preamble = FALSE;
       }
       pktLength = pktRxByteOffset;
    }
  }
}

uint8 sendNotification(uint8* bytes_sent, uint8 len)
{
  attHandleValueNoti_t noti;
  //dummy handle
  noti.handle = 0x2E;
  noti.len = len;
  uint8 i;
  
  for (i= 0; i < len; i++)
  {
    noti.value[i] = bytes_sent[i];
  }
  if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
  {
    return 0xFF;
  } 
  return 0xAA;
}

uint8 sendAckMessage(uint8 bytes_sent)
{
  uint8 data[1] = {0};
  
  //data[0]= SERIAL_MSG_START_ID;
  //data[1]= SERIAL_ACK;
  data[0]= bytes_sent;
  uint8 success_len = HalUARTWrite(NPI_UART_PORT, (uint8*)data, 1);
  if (success_len == 1)
  {
    return SUCCESS;
  }
  else
  {
    return FAILURE;   //ack wasn't sent over UAR
  }
}

uint16 circular_diff(uint16 offset, uint16 tail)
{
  if (offset > tail)
  {
    return (offset - tail);
  }
  else
  {
    return (RX_BUFF_SIZE - tail) + offset;
  }    
}

uint16 circular_add(uint16 x, uint16 y)
{
  uint16 sum = x + y;
  if (sum != RX_BUFF_SIZE)
  {
    sum = sum % RX_BUFF_SIZE;
  }
  else
  {
    sum = 0;
  }
  return sum;
}

void clearRxBuf(void)
{
  NPI_ReadTransport(pktBuf, NPI_UART_RX_BUF_SIZE);
}

void sendCmd(uint8* cmd, int cmd_len)
{
  HalUARTWrite(NPI_UART_PORT, (uint8*)cmd, cmd_len);
}

uint8 sendData(uint16 diff)
{
    //can send max 8 packets per connection interval
    uint8 packets_sent = 0;
    //ensure queue of notification is successful
    bool send_error = FALSE;
    //return value to update tail and send ack to msp
  
    attHandleValueNoti_t noti;      
    //dummy handle
    noti.handle = 0x2E;
    noti.value[0] = 0xA7;
    blePktOffset = 0;
  
    //counter
    uint8 i;
    seqNum = tmpPktIdx*8;
  
    while ((packets_sent < 8) && (send_error == FALSE) && (isLastPkt == 0) )
    {  
        //send 20 bytes
        noti.len = 20;
        uint8 sum = 0xA7;
        noti.value[1] = seqNum & 0xFF;
        noti.value[2] = (seqNum >> 8) & 0xFF;
        sum += noti.value[1];
        sum += noti.value[2];
        for (i = 3; i < noti.len-1; i++)
        {
            noti.value[i] = pktBuf[blePktOffset];
            sum += noti.value[i];
            blePktOffset++;
        }
        noti.value[noti.len-1] = sum;
        //connection handle currently hardcoded
        if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
        {
            seqNum++;
            packets_sent++;
        }
        else
        {
            send_error = TRUE;
        }
    }
    //send remaining bytes  
    while ((packets_sent < 8) && (diff > 0) && (send_error == FALSE) && (isLastPkt == 1))
    {
      if(diff > 16){
        noti.len = 20;
        diff -= 16;
      }
      else{
        noti.len = diff + 4;
        diff = 0;
      }
      uint8 sum = 0xA7;
      noti.value[1] = seqNum & 0xFF;
      noti.value[2] = (seqNum >> 8) & 0xFF;
      sum += noti.value[1];
      sum += noti.value[2];
      for (i = 3; i < noti.len-1; i++)
      {
         noti.value[i] = pktBuf[blePktOffset];
         sum += noti.value[i];
         blePktOffset++;
      }
      noti.value[noti.len-1] = sum;
      if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
      {
          seqNum++;
          packets_sent++;
      }
      else
      {
          send_error = TRUE;
      }
    }
    return packets_sent;
}
