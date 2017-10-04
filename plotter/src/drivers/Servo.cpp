#include <Servo.h>

Servo::Servo() {
    // TODO Auto-generated constructor stub
    Chip_SCT_Init(LPC_SCT0);
    // TODO Auto-generated constructor stub
    LPC_SCT0->CONFIG |= (1 << 17);                      // two 16-bit timers, auto limit
    LPC_SCT0->CTRL_L |= (72-1) << 5;                    // set prescaler, SCTimer/PWM clock = 1 MHz
    LPC_SCT0->MATCHREL[0].L = PWM_freq - 1;             // match 0 @ 1000/1MHz = (1 kHz PWM freq)
    LPC_SCT0->MATCHREL[1].L = PWM_cycle;                // match 1 used for duty cycle (in 10 steps)
    LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF;              // event 0 happens in all states
    LPC_SCT0->EVENT[0].CTRL = (1 << 12);                // match 0 condition only
    LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF;              // event 1 happens in all states
    LPC_SCT0->EVENT[1].CTRL = (1 << 0) | (1 << 12);     // match 1 condition only
    LPC_SCT0->OUT[0].SET = (1 << 0);                    // event 0 will set SCTx_OUT0
    LPC_SCT0->OUT[0].CLR = (1 << 1);                    // event 1 will clear SCTx_OUT0
    LPC_SCT0->CTRL_L &= ~(1 << 2);

}

void Servo::moveServo(double value){

    double duty_cycle = PWM_cycle;
    //int last_value = duty_cycle;
    char debug[60];

    //calculating from bit to hz

    duty_cycle = ((value/255.00)+1.00)*1000.00;

    if (duty_cycle < 1003){duty_cycle = 1000;}
    if (duty_cycle > 1996){duty_cycle = 2000;}

    sprintf(debug, "Duty cycle before Match: %5.2f \n", (double)duty_cycle);
    ITM_write(debug);

    //change duty cycle
    LPC_SCT0->MATCHREL[1].L = duty_cycle;

    sprintf(debug, "Duty cycle final: %5.2f \n", (double)LPC_SCT0->MATCHREL[1].L);
    ITM_write(debug);

    Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, port, pin);

}

Servo::~Servo() {
    // TODO Auto-generated destructor stub
}

