#include "board.h"

#include "SerialLog.h"

#include "semphr.h"
#include "portmacro.h"

SerialLog::SerialLog() {
    serialMutex = xSemaphoreCreateMutex();
}

SerialLog::~SerialLog() {
    if (serialMutex != NULL) {
        vSemaphoreDelete(serialMutex);
    }
}

void SerialLog::write(char *description) {
    if (serialMutex != NULL) {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdPASS) {
            Board_UARTPutSTR(description);
            xSemaphoreGive(serialMutex);
        }
    }
}

int SerialLog::read() {
    if (serialMutex != NULL) {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdPASS) {
            int c = Board_UARTGetChar();
            xSemaphoreGive(serialMutex);
            return c;
        }
    }
}
