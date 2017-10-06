#include <GParser.h>

#include <stdlib.h>

GParser::GParser() {
}

GParser::~GParser() {
}

Command GParser::parse(char *instruction, int len) {
    Command ret;
    ret.type = Command::invalid;

    if (instruction[0] == 'G') {

        int code = strtol(instruction + 1, NULL, 10);

        switch (code) {
        case 1:
            ret.type = Command::move;
            ret.params[0] = findParam('X', instruction, len);
            ret.params[1] = findParam('Y', instruction, len);
            ret.params[2] = findParam('A', instruction, len);

            break;
        case 28:
            ret.type = Command::to_origin;

            break;
        default:
            break;
        }
    } else if (instruction[0] == 'M') {

        int code = strtol(instruction + 1, NULL, 10);

        switch (code) {
        case 1:
            ret.type = Command::pen_position;
            ret.params[0] = strtol(instruction + 2, NULL, 10);

            break;
        case 2:
            ret.type = Command::pen_setting;
            ret.params[0] = findParam('U', instruction, len);
            ret.params[1] = findParam('D', instruction, len);

            break;
        case 4:
            ret.type = Command::laser;
            ret.params[0] = strtol(instruction + 2, NULL, 10);

            break;
        case 5:
            ret.type = Command::plotter_setting;
            ret.params[0] = findParam('A', instruction, len);
            ret.params[1] = findParam('B', instruction, len);
            ret.params[2] = findParam('H', instruction, len);
            ret.params[3] = findParam('W', instruction, len);
            ret.params[4] = findParam('S', instruction, len);

            break;
        case 10:
            ret.type = Command::connected;

            break;
        case 11:
            ret.type = Command::done;
        default:
            break;
        }
    }

    return ret;
}

/*
 * Look for numbers after key and return it
 * If key is not found, return 0
 */
float GParser::findParam(char key, char *instruction, int len) {
    int keyIndex = -1;

    for (int i = 0; i < len; i++)
        if (instruction[i] == key) {
            keyIndex = i;
        }

    if (keyIndex == -1)
        return 0;
    else {
        float ret = strtof(instruction + keyIndex + 1, NULL);

        return ret;
    }
}
