#include <stddef.h>
#include <stdint.h>

#include "StdCrc32.h"

/* Table of CRCs of all 8-bit messages. */
unsigned long crc_table[256];

/* Flag: has the table been computed? Initially false. */
int crc_table_computed = 0;

/* Make the table for a fast CRC. */
void make_crc_table(void) {
    unsigned long c;
    int n, k;
    for (n = 0; n < 256; n++) {
        c = (unsigned long) n;
        for (k = 0; k < 8; k++) {
            if (c & 1) {
                c = 0xedb88320L ^ (c >> 1);
            } else {
                c = c >> 1;
            }
        }
        crc_table[n] = c;
    }
    crc_table_computed = 1;
}

unsigned long update_crc(unsigned long crc, unsigned char *buf, int len) {
    unsigned long c = crc ^ 0xffffffffL;
    int n;

    if (!crc_table_computed) {
        make_crc_table();
    }
    for (n = 0; n < len; n++) {
        c = crc_table[(c ^ buf[n]) & 0xff] ^ (c >> 8);
    }
    return c ^ 0xffffffffL;
}

void stdCrc32Init(StdCrc32 *crc32) {
    crc32->crc = 0;
    crc32->dataSize = 0;
}

void stdCrc32Update(StdCrc32 *crc32, void *data, size_t dataSize) {
    crc32->crc = update_crc(crc32->crc, (unsigned char *) data, dataSize);
}

uint32_t stdCrc32Get(StdCrc32 *crc32) {
    uint8_t buf[4] = { 0, 0, 0, 0 };
    size_t paddingSize = crc32->dataSize % 4;

    if (paddingSize == 0) {
        return crc32->crc;
    } else {
        return update_crc(crc32->crc, buf, paddingSize);
    }
}

