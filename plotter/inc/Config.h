/*
 * Setup.h
 *
 *  Created on: Oct 5, 2017
 *      Author: Nick
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/* X, Y stepper motor */
#define motorXPort      0
#define motorXPin       27
#define motorXDirPort   0
#define motorXDirPin    28

#define motorYPort      0
#define motorYPin       24
#define motorYDirPort   1
#define motorYDirPin    0

#define motorPps        400

/* Limit switches */
#define limitXMinPort   0
#define limitXMinPin    9
#define limitXMaxPort   0
#define limitXMaxPin    29

#define limitYMinPort   0
#define limitYMinPin    0
#define limitYMaxPort   1
#define limitYMaxPin    3

/* Pen */
#define penPort         0
#define penPin          10

/* Plotter setting */
#define plotterWidth    380
#define plotterHeight   310

struct XYSetup {
	int pen_up = 130;
	int pen_down = 90;
	int speed = 50;
	float last_x_pos;
	float last_y_pos;
	int length_x = plotterWidth;
	int length_y = plotterHeight;
};

#endif /* CONFIG_H_ */
