/*
 * Command.h
 *
 *  Created on: Sep 28, 2017
 *      Author: tuanngu
 */

#ifndef COMMAND_H_
#define COMMAND_H_

struct Command {
    enum CType {
        pen_position, // M1
        pen_setting, // M2
        laser, // M4
        plotter_setting, // M5
        connected, // M10
        move, // G1
        to_origin, // G28
        invalid
    };

    CType type;
    float params[6] = {0};
};



#endif /* COMMAND_H_ */
