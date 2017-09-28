/*
 * GParser.h
 *
 *  Created on: Sep 18, 2017
 *      Author: tuanngu
 */

#ifndef GPARSER_H_
#define GPARSER_H_

#include "Command.h"

class GParser {
public:
    GParser();
    virtual ~GParser();
    Command parse(char *instruction, int len);

private:
    float GParser::findParam(char key, char *instruction, int len);
};

#endif /* GPARSER_H_ */
