/*
 * Fmutex.cpp
 *
 *  Created on: 15.8.2017
 *      Author: krl
 */

#include <usb_cdc/Fmutex.h>

Fmutex::Fmutex() {
    // TODO Auto-generated constructor stub
    mutex = xSemaphoreCreateMutex();
}

Fmutex::~Fmutex() {
    // TODO Auto-generated destructor stub
}

void Fmutex::lock() {
    xSemaphoreTake(mutex, portMAX_DELAY);
}

void Fmutex::unlock() {
    xSemaphoreGive(mutex);
}
