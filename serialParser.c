#include "serialParser.h"
#include <assert.h>

static BOOL firstPreamble = FALSE;
static BOOL secondPreamble = FALSE;

extern const BYTE TYPE1;      //ECG
extern const BYTE TYPE2;      //ACC
extern const BYTE TYPE3;      //TMP

void RingBuffer_init(RingBuffer *buffer)
{
    buffer->length  = RX_BUFF_SIZE;
    buffer->start = 0;
    buffer->end = 0;
}

int RingBuffer_write(RingBuffer *buffer, char *data, int length)
{
    if(RingBuffer_available_data(buffer) == 0) {
        buffer->start = buffer->end = 0;
    }
    
    if(length > RingBuffer_available_space(buffer))
        return -1;
    
    unsigned int i;
    int temp = buffer->end;
    for(i = 0; i < length; i++){
        buffer->buffer[temp] = data[i];
        temp++;
        temp %= buffer->length;
    }

    RingBuffer_commit_write(buffer, length);
    return length;
}

int RingBuffer_read(RingBuffer *buffer, char *target, int amount)
{
    if(amount > RingBuffer_available_data(buffer))
        return -1;

    unsigned int i;
    int temp = buffer->start;
    for(i = 0; i < amount; i++){
        target[i] = buffer->buffer[temp];
        temp++;
        temp %= buffer->length;
    }

    RingBuffer_commit_read(buffer, amount);
    if(buffer->end == buffer->start) {
        buffer->start = buffer->end = 0;
    }

    return amount;
}


