#ifndef _CMD_PARSER_H_
#define _CMD_PARSER_H_

enum{
    RED=0,
    GREEN,
    BLUE,
    WHITE
};

int cmd_parser(uint8_t *buf);

#endif
