#include <GParser.h>

#include <stdlib.h>
#include <string.h>

GParser::GParser() {
}

GParser::~GParser() {
}

Command GParser::parse(char *instruction, int len) {
    Command ret;
    ret.type = Command::invalid;

    char *result;

    if ((result = strchr(instruction, 'G')) != NULL ) {

        int code = strtol(result + 1, NULL, 10);

        switch (code) {
        case 1:
            ret.type = Command::move;
            ret.params[0] = findParam('X', result, len);
            ret.params[1] = findParam('Y', result, len);
            ret.params[2] = findParam('A', result, len);

            break;
        case 28:
            ret.type = Command::to_origin;

            break;
        default:
            break;
        }
    } else if ((result = strchr(instruction, 'M')) != NULL) {

        int code = strtol(result + 1, NULL, 10);

        switch (code) {
        case 1:
            ret.type = Command::pen_position;
            ret.params[0] = strtol(result + 2, NULL, 10);

            break;
        case 2:
            ret.type = Command::pen_setting;
            ret.params[0] = findParam('U', result, len);
            ret.params[1] = findParam('D', result, len);

            break;
        case 4:
            ret.type = Command::laser;
            ret.params[0] = strtol(result + 2, NULL, 10);

            break;
        case 5:
            ret.type = Command::plotter_setting;
            ret.params[0] = findParam('A', result, len);
            ret.params[1] = findParam('B', result, len);
            ret.params[2] = findParam('H', result, len);
            ret.params[3] = findParam('W', result, len);
            ret.params[4] = findParam('S', result, len);

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
