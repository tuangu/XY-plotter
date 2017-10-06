/*
 * Laser.h
 *
 *  Created on: Sep 18, 2017
 *      Author: tuanngu
 */

#ifndef LASER_H_
#define LASER_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#include "sct_15xx.h"
#else
#include "board.h"
#endif
#endif
#include "FreeRTOS.h"
#define PWM_FREQ 1000
#define PWM_CYCLE 255

class Laser {
public:
    Laser(int port = 0, int pin = 12);
    virtual ~Laser();
    void changeLaserPower(int pow);
private:
    int port;
    int pin;
    int m_pow = 0;
};

#endif /* LASER_H_ */
