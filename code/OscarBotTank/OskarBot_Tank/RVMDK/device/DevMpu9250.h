#include <stdint.h>

#include "inv_mpu.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"

#define MPU_FREQ_HZ_DEFAULT     (100)
#define COMPASS_FREQ_HZ_DEFAULT (100)

extern inv_error_t devMpu9250Status;
extern uint8_t devMpu9250GyroUpdated;
extern uint8_t devMpu9250CompassUpdated;

inv_error_t devMpu9250Init(void);
inv_error_t devMpu9250Loop(void);

int devMpu9250GetAccelFloat(float *data, int8_t *accuracy, inv_time_t *timestamp);
int devMpu9250GetGyroFloat(float *data, int8_t *accuracy, inv_time_t *timestamp);
int devMpu9250GetCompassFloat(float *data, int8_t *accuracy, inv_time_t *timestamp);
int devMpu9250GetQuatFloat(float *data, int8_t *accuracy, inv_time_t *timestamp);
int devMpu9250GetEulerFloat(float *data, int8_t *accuracy, inv_time_t *timestamp);
int devMpu9250GetRotMatFloat(float *data, int8_t *accuracy, inv_time_t *timestamp);
int devMpu9250GetHeadingFloat(float *data, int8_t *accuracy, inv_time_t *timestamp);
