#include "../include/mqueue.h"
#include <string.h>
//On success, mq_open() returns a message queue descriptor for use by other message queue functions.  On error, mq_open() returns (mqd_t) -1, with errno set to indicate the error.
mqd_t mq_open(const char *name, int oflag){
#if defined(OS_FREERTOS)
    mqd_t ret = xQueueCreate(oflag, 1);
    return ret;
#elif defined(OS_UCOS)
    OS_Q *q;
    OS_ERR err;
    LIB_ERR l_err;
    q = (OS_Q*)Mem_SegAlloc(0, 0, sizeof(OS_Q), &l_err);
    OSQCreate(q, "", oflag, &err);
    return q;
#endif
}
//On success mq_close() returns 0; on error, -1 is returned, with errno set to indicate the error.
int mq_close(mqd_t mqdes){
#if defined(OS_FREERTOS)
    vQueueDelete(mqdes);
#elif defined(OS_UCOS)

#endif
    return 0;
}
//On success, mq_receive() and mq_timedreceive() return the number of bytes in the received message; on error, -1 is returned, with errno set to indicate the error.
int mq_timedreceive(mqd_t mqdes, char *msg_ptr, size_t msg_len, unsigned *msg_prio, const struct timespec *abs_timeout){
    int ret = 0;
#if defined(OS_FREERTOS)
    TickType_t timeout = abs_timeout->tv_sec * portTICK_PERIOD_MS;
    timeout += abs_timeout->tv_nsec / 1000000 * portTICK_PERIOD_MS;
    while(msg_len){
        if(xQueueReceive(mqdes, msg_ptr, timeout) != pdTRUE){
            break;
        }
        msg_ptr++;
        ret++;
        msg_len--;
    }
#elif defined(OS_UCOS)
    uint8_t *pu8val;
    OS_ERR err;
    OS_MSG_SIZE p_msg_size;
    OS_TICK timeout = abs_timeout->tv_sec * TICK_RATE_HZ;
    timeout += abs_timeout->tv_nsec / 1000000 * TICK_RATE_HZ;
    p_msg_size = msg_len;
    pu8val = OSQPend(mqdes, timeout, OS_OPT_PEND_BLOCKING, &p_msg_size, 0, &err);
    if(err == OS_ERR_NONE && pu8val){
        memcpy_s(msg_ptr, pu8val, p_msg_size, msg_len);
        ret = p_msg_size;
    }
#endif
    return ret;
}
//On success, mq_send() and mq_timedsend() return zero; on error, -1 is returned, with errno set to indicate the error.
int mq_send(mqd_t mqdes, const char *msg_ptr, size_t msg_len, unsigned msg_prio){
#if defined(OS_UCOS)
    OS_ERR err;
    OSQPost(mqdes, (void*)msg_ptr, msg_len, OS_OPT_POST_ALL | OS_OPT_POST_FIFO, &err);
    return msg_len;
#endif
#if defined(OS_FREERTOS)

    while(msg_len > 0){
        if(xQueueSend(mqdes, msg_ptr, portMAX_DELAY) != pdTRUE) break;
        msg_len --;
        msg_ptr++;
    }
    return 0;
#endif
}
// end of file
