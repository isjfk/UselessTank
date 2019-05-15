#include "Math.h"

inline int iabs(int value) {
    if (value < 0) {
        return -value;
    } else {
        return value;
    }
}

inline int irange(int value, int min, int max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

inline float frange(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}
