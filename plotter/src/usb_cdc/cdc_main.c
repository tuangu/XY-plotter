/*
 * @brief Virtual communication port example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include <stdio.h>
#include <string.h>
#include <usb_cdc/app_usbd_cfg.h>
#include <usb_cdc/cdc_vcom.h>
#include <usb_cdc/ITM_write.h>
#include <usb_cdc/user_vcom.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

static USBD_HANDLE_T g_hUsb;
static uint8_t g_rxBuff[RCV_BUFFERS][RCV_BUFSIZE];
const USBD_API_T *g_pUsbApi;
typedef struct {
    uint8_t *dptr;
    uint32_t dlen;
} SendItem;

static SemaphoreHandle_t xCDCEventSemaphore;
static SemaphoreHandle_t SendMutex;
static SemaphoreHandle_t SendComplete;
static QueueHandle_t SendQueue;
static QueueHandle_t AllocQueue;
static QueueHandle_t ReceiveQueue;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void USB_send(uint8_t *data, uint32_t length) {
    SendItem send = { data, length };
    xSemaphoreTake(SendMutex, portMAX_DELAY); // take mutex

    xQueueSend(SendQueue, &send, portMAX_DELAY); // send data to queue
    xSemaphoreGive(xCDCEventSemaphore); // wake up the task
    xSemaphoreTake(SendComplete, portMAX_DELAY); // wait until send is complete

    xSemaphoreGive(SendMutex);
}

uint32_t USB_receive(uint8_t *data, uint32_t length) {
    SendItem rec;
    uint32_t len = 0;

    xQueueReceive(ReceiveQueue, &rec, portMAX_DELAY);
    /* copy data to user */
    len = rec.dlen > length ? length : rec.dlen;
    memcpy(data, rec.dptr, len);
    /* send buffer back to allocation queue */
    xQueueSend(AllocQueue, &rec, portMAX_DELAY);

    return len;
}

/**
 * @brief	Handle interrupt from USB0
 * @return	Nothing
 */
void USB_IRQHandler(void) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    USBD_API->hw->ISR(g_hUsb);
    xSemaphoreGiveFromISR(xCDCEventSemaphore, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/* Find the address of interface descriptor for given class type. */
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc,
        uint32_t intfClass) {
    USB_COMMON_DESCRIPTOR *pD;
    USB_INTERFACE_DESCRIPTOR *pIntfDesc = 0;
    uint32_t next_desc_adr;

    pD = (USB_COMMON_DESCRIPTOR *) pDesc;
    next_desc_adr = (uint32_t) pDesc;

    while (pD->bLength) {
        /* is it interface descriptor */
        if (pD->bDescriptorType == USB_INTERFACE_DESCRIPTOR_TYPE) {

            pIntfDesc = (USB_INTERFACE_DESCRIPTOR *) pD;
            /* did we find the right interface descriptor */
            if (pIntfDesc->bInterfaceClass == intfClass) {
                break;
            }
        }
        pIntfDesc = 0;
        next_desc_adr = (uint32_t) pD + pD->bLength;
        pD = (USB_COMMON_DESCRIPTOR *) next_desc_adr;
    }

    return pIntfDesc;
}

/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
void cdc_task(void *pvParameters) {
    USBD_API_INIT_PARAM_T usb_param;
    USB_CORE_DESCS_T desc;
    ErrorCode_t ret = LPC_OK;
    uint32_t rdCnt = 0;
    SendItem snd = { NULL, 0 };
    SendItem rcv = { NULL, 0 };

    /* create semaphores, queues and mutexes */
    xCDCEventSemaphore = xSemaphoreCreateBinary();
    SendQueue = xQueueCreate(1, sizeof(SendItem));
    SendMutex = xSemaphoreCreateMutex();
    SendComplete = xSemaphoreCreateBinary();
    ReceiveQueue = xQueueCreate(RCV_BUFFERS, sizeof(SendItem));
    AllocQueue = xQueueCreate(RCV_BUFFERS, sizeof(SendItem));
    for (int i = 0; i < RCV_BUFFERS; i++) {
        SendItem bf;
        bf.dptr = g_rxBuff[i];
        bf.dlen = 0;
        xQueueSend(AllocQueue, &bf, 0);
    }

    /* register them so that debugger can see them */
    vQueueAddToRegistry(xCDCEventSemaphore, "CDCEvent");
    vQueueAddToRegistry(SendQueue, "SQueue");
    vQueueAddToRegistry(SendMutex, "SMutex");
    vQueueAddToRegistry(SendComplete, "Complete");
    vQueueAddToRegistry(ReceiveQueue, "Receive");
    vQueueAddToRegistry(AllocQueue, "Allocate");

    /* enable clocks */
    Chip_USB_Init();

    /* initialize USBD ROM API pointer. */
    g_pUsbApi = (const USBD_API_T *) LPC_ROM_API->pUSBD;

    /* initialize call back structures */
    memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
    usb_param.usb_reg_base = LPC_USB0_BASE;
    /*	WORKAROUND for artf44835 ROM driver BUG:
     Code clearing STALL bits in endpoint reset routine corrupts memory area
     next to the endpoint control data. For example When EP0, EP1_IN, EP1_OUT,
     EP2_IN are used we need to specify 3 here. But as a workaround for this
     issue specify 4. So that extra EPs control structure acts as padding buffer
     to avoid data corruption. Corruption of padding memory doesn’t affect the
     stack/program behaviour.
     */
    usb_param.max_num_ep = 3 + 1;
    usb_param.mem_base = USB_STACK_MEM_BASE;
    usb_param.mem_size = USB_STACK_MEM_SIZE;

    /* Set the USB descriptors */
    desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
    desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
    /* Note, to pass USBCV test full-speed only devices should have both
     descriptor arrays point to same location and device_qualifier set to 0.
     */
    desc.high_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
    desc.full_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
    desc.device_qualifier = 0;

    /* USB Initialization */
    ret = USBD_API->hw->Init(&g_hUsb, &desc, &usb_param);
    if (ret == LPC_OK) {

        /* Init VCOM interface */
        ret = vcom_init(g_hUsb, &desc, &usb_param);
        if (ret == LPC_OK) {
            /*  enable USB interrupts */
            NVIC_EnableIRQ(USB0_IRQn);
            /* now connect */
            USBD_API->hw->Connect(g_hUsb, 1);
        }

    }

    ITM_write("\r\nUSB CDC class based virtual Comm port example!\r\n");

    do {
#ifndef POLLING_CDC
        /* Sleep until next IRQ happens */
        xSemaphoreTake(xCDCEventSemaphore, configTICK_RATE_HZ / 10);
#endif
    } while (vcom_connected() == 0);

    ITM_write("Connected\r\n");

    while (1) {
        /* try allocate receive buffer if we have none */
        if (rcv.dptr == NULL) {
            if (xQueueReceive(AllocQueue, &rcv, 0) == pdFALSE)
                rcv.dptr = NULL;
        }
        /* Read data if we have buffer */
        if (rcv.dptr != NULL) {
            /* get data. return value tells how much we got. */
            rdCnt = vcom_bread(rcv.dptr, RCV_BUFSIZE);
            if (rdCnt > 0) {
                /* we got data send the data to receive queue */
                rcv.dlen = rdCnt;
                if (xQueueSend(ReceiveQueue, &rcv, 0) == pdTRUE) {
                    rcv.dptr = NULL; /* send succeeded clear pointer to allocate new buffer for next round */
                }
            }
        }
        if (snd.dlen > 0) {
            uint32_t sndCnt = vcom_write(snd.dptr, snd.dlen);
            //DEBUGSTR("Send\r\n");
            snd.dptr += sndCnt;
            snd.dlen -= sndCnt;
            if (snd.dlen == 0) {
                xSemaphoreGive(SendComplete); // notify completion
            }
        }
#ifndef POLLING_CDC
        /* Sleep until next IRQ happens */
        xSemaphoreTake(xCDCEventSemaphore, configTICK_RATE_HZ / 50);
#endif
        /* If we have no pending data to send check if there is data in the queue */
        if (snd.dlen == 0) {
            /* poll for new data - we need to poll because the USB example
             * from NXP was not originally written for FreeRTOS
             */
            if (xQueueReceive(SendQueue, &snd, 0) != pdTRUE)
                snd.dlen = 0;
        }
    }
}
