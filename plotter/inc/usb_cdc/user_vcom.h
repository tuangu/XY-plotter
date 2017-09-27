/*
 * user_vcom.h
 *
 *  Created on: 5.9.2017
 *      Author: krl
 */

#ifndef USER_VCOM_H_
#define USER_VCOM_H_

/* remove comment on the following line to make CDC task to poll without yielding/blocking */
#define POLLING_CDC

/* number of USB receive buffers */
#define RCV_BUFFERS 16
/* size of receive buffers */
#define RCV_BUFSIZE 64


/* this is needed to start CDC USB task. Since CDC functions are written in C we need
 * extern "C" - declaration to tell C++ compiler to use C naming convention for this function.
 */
#ifdef __cplusplus
extern "C"
{
#endif

/* Virtual com port main task */
void cdc_task(void *pvParameters);

/* Writes data to USB com port, maximum length not restricted */
void USB_send(uint8_t *data, uint32_t length);

/* reads data from USB com port.
 * Length must be at least RCV_BUFSIZE
 * If length is smaller some data may be discarded.
 * Function check sizes and never overflows buffers and discards excess data */
uint32_t USB_receive(uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif


#endif /* USER_VCOM_H_ */
