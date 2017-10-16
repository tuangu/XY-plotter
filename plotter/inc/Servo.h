#ifndef SERVO_H_
#define SERVO_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#include "sct_15xx.h"
#else
#include "board.h"
#endif
#endif

#include "usb_cdc/ITM_write.h"
#include "FreeRTOS.h"

#define PWM_freq 20000
#define PWM_cycle 1000;

class Servo {
public:
    Servo(int port = 0, int pin = 8);
    void moveServo(double value);
    virtual ~Servo();
private:
    int port;
    int pin;

    int times;
    double remain;
    double last_value;
    double duty_cycle;
};

#endif /* SERVO_H_ */
