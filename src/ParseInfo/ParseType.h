#ifndef _PARSE_TYPE_H
#define _PARSE_TYPE_H

enum ParseType {
    PARSE_TYPE_INT8,
    PARSE_TYPE_UINT8,

    PARSE_TYPE_INT16,
    PARSE_TYPE_UINT16,

    PARSE_TYPE_INT32,
    PARSE_TYPE_UINT32,

    PARSE_TYPE_FLOAT,
    PARSE_TYPE_DOUBLE,
};

extern const int parseTypeSizes[];

#endif