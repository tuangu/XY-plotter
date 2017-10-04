/*
 * Laser.cpp
 *
 *  Created on: Sep 18, 2017
 *      Author: tuanngu
 */

#include <Laser.h>

Laser::Laser() {
    // TODO Auto-generated constructor stub
	Chip_SCT_Init(LPC_SCT1);
	LPC_SCT1->CONFIG |= (1 << 17); // two 16-bit timers, auto limit
	LPC_SCT1->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock = 1 MHz
	LPC_SCT1->MATCHREL[0].L = PWM_FREQ-1; // match 0 @ 10/1MHz = 10 usec (100 kHz PWM freq)
	LPC_SCT1->MATCHREL[1].L = PWM_CYCLE*PWM_FREQ/PWM_CYCLE; // match 1 used for duty cycle (in 10 steps)
	LPC_SCT1->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all states
	LPC_SCT1->EVENT[0].CTRL = (1 << 12); // match 0 condition only
	LPC_SCT1->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
	LPC_SCT1->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 condition only
	LPC_SCT1->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT0
	LPC_SCT1->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT0
	LPC_SCT1->CTRL_L &= ~(1 << 2); // unhalt it by clearing bit 2 of CTRL reg
}

Laser::~Laser() {
    // TODO Auto-generated destructor stub
}

void Laser::changeLaserPower(int pow){
	if(pow < 0){
		m_pow = 0;
	}else if(pow > 255){
		m_pow = 255;
	}else{
		m_pow = pow;
	}

	LPC_SCT1->MATCHREL[1].L = m_pow*PWM_FREQ/PWM_CYCLE;
	Chip_SWM_MovablePortPinAssign(SWM_SCT1_OUT0_O, port, pin);
}

