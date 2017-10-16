#include <Servo.h>

#include <stdlib.h>
#include <math.h>

Servo::Servo(int port_, int pin_): port(port_), pin(pin_) {

    Chip_SCT_Init(LPC_SCT0);

    LPC_SCT0->CONFIG |= (1 << 17);              // two 16-bit timers, auto limit
    LPC_SCT0->CTRL_L |= (72 - 1) << 5;          // set prescaler, SCTimer/PWM clock = 1 MHz
    LPC_SCT0->MATCHREL[0].L = PWM_freq - 1;     // match 0 @ 1000/1MHz = (1 kHz PWM freq)
    LPC_SCT0->MATCHREL[1].L = PWM_cycle;        // match 1 used for duty cycle (in 10 steps)
    LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF;      // event 0 happens in all states
    LPC_SCT0->EVENT[0].CTRL = (1 << 12);        // match 0 condition only
    LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF;      // event 1 happens in all states
    LPC_SCT0->EVENT[1].CTRL = (1 << 0) | (1 << 12);    // match 1 condition only
    LPC_SCT0->OUT[0].SET = (1 << 0);            // event 0 will set SCTx_OUT0
    LPC_SCT0->OUT[0].CLR = (1 << 1);            // event 1 will clear SCTx_OUT0
    LPC_SCT0->CTRL_L &= ~(1 << 2);              // start
}

void Servo::moveServo(double value) {

    char debug[60]={'0'};

    //calculating from bit to hz
    duty_cycle = ((last_value / 255.00) + 1.00) * 1000.00;

    //get the times and remain to increase or decrease duty cycle step by step
    times = (value > last_value) ? ((value - last_value) / 5) : ((last_value - value) / 5);
    remain = (value > last_value) ? (fmod((value - last_value), 5)) : (fmod((last_value - value), 5));

    sprintf(debug, "Duty cycle before Match: %5.2f \n", (double) duty_cycle);
    ITM_write(debug);

    //change duty cycle

    if(value > last_value){
    	for(int i = 0; i < times; i++){
    	    duty_cycle += ((5 / 255.00)*1000.00);
    	    LPC_SCT0->MATCHREL[1].L = duty_cycle;
    	    Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, port, pin);

    	    sprintf(debug, "Increase to : %5.2f \n", (double) duty_cycle);
    	    ITM_write(debug);
    	 }

    	duty_cycle += ((remain / 255.00)*1000.00);

    	//check duty cycle
		if (duty_cycle < 1003) {
			duty_cycle = 1000;
		}
		if (duty_cycle > 1996) {
			duty_cycle = 2000;
		}
		LPC_SCT0->MATCHREL[1].L = duty_cycle;
    	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, port, pin);
    }

    if(value < last_value){
		for(int i = 0; i < times; i++){
			duty_cycle -= ((5 / 255.00)*1000.00);
			LPC_SCT0->MATCHREL[1].L = duty_cycle;
			Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, port, pin);

			sprintf(debug, "Reduce to : %5.2f \n", (double) duty_cycle);
			ITM_write(debug);
		 }
		duty_cycle -= ((remain / 255.00)*1000.00);
		if (duty_cycle < 1003) {
			duty_cycle = 1000;
		}
		if (duty_cycle > 1996) {
			duty_cycle = 2000;
		}
		LPC_SCT0->MATCHREL[1].L = duty_cycle;
		Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, port, pin);
	}

    sprintf(debug, "Final cycle: %5.2f, times: %d, remain: %.2f\n", (double) LPC_SCT0->MATCHREL[1].L, (int)times, (double)remain);
    ITM_write(debug);

    last_value = value;
}

Servo::~Servo() {
}
