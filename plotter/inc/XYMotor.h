#ifndef XYMOTOR_H_
#define XYMOTOR_H_

#include "DigitalIoPin.h"
#include "FreeRTOS.h"
#include "semphr.h"

class XYMotor {
public:
    XYMotor(DigitalIoPin* dirX, DigitalIoPin* stepX, DigitalIoPin* dirY, DigitalIoPin* stepY);
    virtual ~XYMotor();

    void setPps(int newPps);
    int getPps();
    void setBaseX(int baseX);
    void setTotalStepX(int totalStepX);
    void setBaseY(int baseY);
    void setTotalStepY(int totalStepY);

    void move(float fromX, float fromY, float toX, float toY);
    bool irqHandler();
    void RIT_start(int pps); // pps = pulse per revolution
private:
    DigitalIoPin* dirXPin;
    DigitalIoPin* stepXPin;
    DigitalIoPin* dirYPin;
    DigitalIoPin* stepYPin;

    int x;
    int y;
    int stepX;
    int stepY;
    bool xState;
    bool yState;
    int delta;
    bool isUpdateDelta;
    bool motorYMove;
    int RIT_count;
    SemaphoreHandle_t sbRIT;

    int pps;
    int totalStepX;
    int totalStepY;
    int baseX;
    int baseY;
    bool dirToOrigin = 0;
};

#endif /* XYMOTOR_H_ */

//Polyline move(float x0, float y0, float x1, float y1) {
//    Polyline ret(Stroke(.5, Color::Blue));
//
//    float dx = x1 - x0;
//    float dy = y1 - y0;
//
//    float xsign = (dx > 0) ? 1 : -1;
//    float ysign = (dy > 0) ? 1 : -1;
//
//    dx = fabsf(dx);
//    dy = fabsf(dy);
//
//    float xx, xy, yx, yy;
//    if (dx > dy) {
//        xx = xsign;
//        xy = yx = 0;
//        yy = ysign;
//    } else {
//        float temp = dx;
//        dx = dy;
//        dy = temp;
//
//        xx = yy = 0;
//        xy = ysign;
//        yx = xsign;
//    }
//
//    float D = 2*dy - dx;
//    float y = 0;
//    float x = 0;
//
//    for (float i = 0; i <= dx; i += 0.01) {
//        Point po(x0 + i*xx + y*yx, y0 + i*xy + y*yy);
//        ret << po;
//
//        if (D > 0) {
//            y += 0.01;
//            D -= dx;
//        }
//        D += dy;
//    }
//
//    return ret;
//}
