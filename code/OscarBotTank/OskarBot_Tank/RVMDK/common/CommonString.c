#include "CommonString.h"

uint8_t strIsStartWith(unsigned char *string, unsigned char *prefix) {
    while (*prefix) {
        if (*string++ != *prefix++) {
            return 0;
        }
    }
    return 1;
}
