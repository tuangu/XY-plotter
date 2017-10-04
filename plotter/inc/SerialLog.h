#ifndef SERIALLOG_H_
#define SERIALLOG_H_

#include "FreeRTOS.h"
#include "semphr.h"

class SerialLog {
public:
    SerialLog();
    virtual ~SerialLog();
    void write(char *description);
    int read();
private:
    SemaphoreHandle_t serialMutex;
};

#endif /* SERIALLOG_H_ */
