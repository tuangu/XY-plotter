#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#define USING_USB_CDC   0

#include <cr_section_macros.h>

#include "string.h"
#include "math.h"

#include "FreeRTOS.h"
#include "user_vcom.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "SerialLog.h"
#include "GParser.h"
#include "Laser.h"
#include "Servo.h"
#include "StepperMotor.h"
#include "Command.h"
#include "Setup.h"

void setupHardware();

/* Task Declaration */
void vReceiveTask(void *vParameters);
void vExecuteTask(void *vParameters);
void vCalibrateTask(void *vParameters);

StepperMotor *xmotor;
StepperMotor *ymotor;
Servo *pen;

Setup XYSetup;
SerialLog debugSerial;
QueueHandle_t qCommand;
EventGroupHandle_t eMotor;

int main(void) {

    setupHardware();

    pen = new Servo();

    eMotor = xEventGroupCreate();
    qCommand = xQueueCreate(1, sizeof(Command));

    xTaskCreate(vReceiveTask, "Receive Task", configMINIMAL_STACK_SIZE * 2, NULL,
            (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);

    xTaskCreate(vExecuteTask, "Execute Task", configMINIMAL_STACK_SIZE * 4, NULL,
            (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);

    xTaskCreate(vCalibrateTask, "Calibrate Task", configMINIMAL_STACK_SIZE,
            NULL, (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);

#if USING_USB_CDC
    xTaskCreate(cdc_task, "CDC Task", configMINIMAL_STACK_SIZE * 2, NULL,
            (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);
#endif
    vTaskStartScheduler();

    while (1)
        ;

    return 0;
}

void setupHardware() {
    SystemCoreClockUpdate();
    Board_Init();

    ITM_init();

    Board_LED_Set(0, false);
}

/***** Task Definition *****/

void vCalibrateTask(void *vParameters) {
    xmotor = new StepperMotor(LPC_SCT2, 60, 0, 24, 1, 0, 0, 27, 0, 28, &eMotor, motorEventX);
    xmotor->calibrate();
    ymotor = new StepperMotor(LPC_SCT3, 60, 0, 9, 0, 29, 1, 9, 1, 10, &eMotor, motorEventY);
    ymotor->calibrate();

    XYSetup.last_x_pos = xmotor->getCurrentPos();
    XYSetup.last_y_pos = ymotor->getCurrentPos();
    XYSetup.speed = 50;
    XYSetup.length_x = 380; // = 290 if using simulator
    XYSetup.length_y = 310; // = 290 if using simulator
    XYSetup.pen_down = 90;
    XYSetup.pen_up = 130;

    xmotor->setBaseLength(XYSetup.length_x);
    ymotor->setBaseLength(XYSetup.length_y);

    vTaskDelete(NULL);
}

void vReceiveTask(void *vParameters) {
    GParser parser;
    char buffer[32];
    int idx = 0;

    while (1) {
        /* get GCode from mDraw */
#if USING_USB_CDC
        char buffer[RCV_BUFSIZE] = {'0'};
        int idx = 0;
        while (1) {
            int len = USB_receive((uint8_t *) (buffer + idx), RCV_BUFSIZE);

            char *pos = strstr((buffer + idx), "\n");// find '\n' <=> end of command

            if (pos != NULL)
                break;

            idx += len;
        }

        debugSerial.write(buffer);
        debugSerial.write((char *) "\r");

        /* parse GCode */
        Command gcode = parser.parse(buffer, strlen(buffer));

        /* send commands into queue */
        xQueueSendToBack(qCommand, &gcode, portMAX_DELAY);
#else   // use usb debug
        int c;
        if ((c = debugSerial.read()) != EOF) {
            buffer[idx++] = c;
            if (c == '\n') {
                buffer[idx] = '\0';
                ITM_write(buffer);

                Command gcode = parser.parse(buffer, strlen(buffer));
                xQueueSendToBack(qCommand, &gcode, portMAX_DELAY);

                memset(buffer, '\0', 32);
                idx = 0;
            }
        }
#endif
    }
}

void vExecuteTask(void *vParameters) {

    EventBits_t eventBit;
    Command recv;

    while (1) {
        /* wait for commands from queue */
        xQueueReceive(qCommand, &recv, portMAX_DELAY);

        /* do something useful*/
        switch (recv.type) {
        case Command::connected:
            // debugSerial.write("M10 XY 290 290 0.00 0.00 A0 B0 H0 S50 U130 D90 \n");
            {
                char buffer[48];
                snprintf(buffer, 48, "M10 XY %d %d 0.00 0.00 A0 B0 H0 S%d U%d D%d \n",
                         XYSetup.length_x, XYSetup.length_y, XYSetup.speed, XYSetup.pen_up, XYSetup.pen_down);
                debugSerial.write(buffer);
                ITM_write(buffer);
            }
            break;
        case Command::laser:
            // TODO laser

            break;
        case Command::move:
            /* calculate new rpm*/
            {
                float deltaX = xmotor->getMoveCount(recv.params[0]);
                float deltaY = ymotor->getMoveCount(recv.params[1]);

                float rpmX = (deltaY > 0) ? fabs(deltaX / deltaY * motorBaseRpm) : motorBaseRpm;
                xmotor->setRpm(rpmX * XYSetup.speed / 100);
                ymotor->setRpm(motorBaseRpm * XYSetup.speed / 100);
            }
            /* move */
            xmotor->move(recv.params[0]);
            ymotor->move(recv.params[1]);

            eventBit = xEventGroupWaitBits(eMotor, motorEventX || motorEventY, pdTRUE, pdTRUE, portMAX_DELAY);
            break;
        case Command::pen_position:
            pen->moveServo(recv.params[0]);
            break;
        case Command::pen_setting:
            XYSetup.pen_up = recv.params[0];
            XYSetup.pen_down = recv.params[1];
            break;
        case Command::plotter_setting:
            XYSetup.length_y = recv.params[2];
            XYSetup.length_x = recv.params[3];
            XYSetup.speed = recv.params[4];
            break;
        case Command::to_origin:
            xmotor->setRpm(60);
            ymotor->setRpm(60);
            xmotor->move(0);
            ymotor->move(0);
            break;
        case Command::done:
            debugSerial.write("M11 0 0 0 0");
            break;
        case Command::invalid:
        default:
            break;
        }

        /* send 'OK' back to mDraw */
#if USING_USB_CDC
        USB_send((uint8_t *) message, 4);
#else
        debugSerial.write("OK\n");
#endif

    }
}

/* the following is required if runtime statistics are to be collected */
extern "C" {

/* X axis motor */
void SCT2_IRQHandler(void) {
    portEND_SWITCHING_ISR(xmotor->irqHandler());
}

/* Y axis motor */
void SCT3_IRQHandler(void) {
    portEND_SWITCHING_ISR(ymotor->irqHandler());
}

void vConfigureTimerForRunTimeStats(void) {
    Chip_SCT_Init(LPC_SCTSMALL1);
    LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
    LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}
/* end runtime statistics collection */
