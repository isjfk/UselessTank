#include <stddef.h>

typedef struct {
    float gyro[3];
    float accel[3];
    float compass[3];
    float quat[4];
    uint32_t motorEncoderLeft;
    uint32_t motorEncoderRight;
} TankMsgData;

typedef struct {
    uint64_t startTag;
    uint32_t header;
    uint32_t dataLength;
    TankMsgData data;
    uint32_t crc;
} TankMsg;

