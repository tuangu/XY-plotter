#ifndef GPARSER_H_
#define GPARSER_H_

#include "Command.h"

class GParser {
public:
    GParser();
    virtual ~GParser();
    Command parse(char *instruction, int len);

private:
    float findParam(char key, char *instruction, int len);
};

#endif /* GPARSER_H_ */
