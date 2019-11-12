#include "DevMpu9250.h"
#include "system/SysIrq.h"
#include "DevMpu9250_board.h"

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = {  0, -1,  0,
                      1,  0,  0,
                      0,  0,  1 }
};

static struct platform_data_s compass_pdata = {
    .orientation = { -1,  0,  0,
                      0,  1,  0,
                      0,  0, -1}
};

inv_error_t devMpu9250Status = -1;
uint8_t devMpu9250GyroUpdated = 0;
uint8_t devMpu9250CompassUpdated = 0;

inv_time_t prevGyroSysTickMs = 0;
inv_time_t prevCompassSysTickMs = 0;

/**
 * Initialize MPU9250.
 */
inv_error_t devMpu9250Init(void) {
    struct int_param_s int_param;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned short compass_fsr;
    inv_time_t currSysTickMs;

    // Wait 500ms to wait for device stable.
    sysDelayMs(500);

    IIC_Init();

    devMpu9250Status = mpu_init(&int_param);
    if (devMpu9250Status) {
        return devMpu9250Status;
    }

    devMpu9250Status = inv_init_mpl();
    if (devMpu9250Status) {
        return devMpu9250Status;
    }

    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    /* The MPL expects compass data at a constant rate (matching the rate
     * passed to inv_set_compass_sample_rate). If this is an issue for your
     * application, call this function, and the MPL will depend on the
     * timestamps passed to inv_build_compass instead.
     *
     * inv_9x_fusion_use_timestamps(1);
     */

    /* This function has been deprecated.
     * inv_enable_no_gyro_fusion();
     */

    /* Update gyro biases when not in motion.
     * WARNING: These algorithms are mutually exclusive.
     */
    inv_enable_fast_nomot();
    /* inv_enable_motion_no_motion(); */
    /* inv_set_no_motion_time(1000); */

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();

    /* This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test (see case 't' in
     * handle_input), but this algorithm can be enabled if the self-test can't
     * be executed in your application.
     */
    inv_enable_in_use_auto_calibration();

    /* Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();

    /* If you need to estimate your heading before the compass is calibrated,
     * enable this algorithm. It becomes useless after a good figure-eight is
     * detected, so we'll just leave it out to save memory.
     * inv_enable_heading_from_gyro();
     */

    /* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();

    devMpu9250Status = inv_start_mpl();
    if (devMpu9250Status) {
        return devMpu9250Status;
    }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(MPU_FREQ_HZ_DEFAULT);

    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(COMPASS_FREQ_HZ_DEFAULT);

    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    mpu_get_compass_fsr(&compass_fsr);

    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);

    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(1000000L / COMPASS_FREQ_HZ_DEFAULT);

    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);

    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);

    // Wait 500ms for device stabilize.
    sysDelayMs(500);
    get_tick_count(&currSysTickMs);
    prevGyroSysTickMs = currSysTickMs;
    prevCompassSysTickMs = currSysTickMs;

    devMpu9250Status = 0;
    return devMpu9250Status;
}

/**
 * Run a MPU9250 mpl data build loop.
 * Call this function perodically if you want get data from mpl library.
 */
inv_error_t devMpu9250Loop(void) {
    short reg[3];
    long data[3];
    inv_time_t timestamp;
    inv_error_t status = 0;
    uint8_t mpl_updated = 0;
    inv_time_t currSysTickMs;

    devMpu9250GyroUpdated = 0;
    devMpu9250CompassUpdated = 0;

    if (devMpu9250Status) {
        return devMpu9250Status;
    }

    get_tick_count(&currSysTickMs);
    if ((currSysTickMs - prevGyroSysTickMs) >= (1000 / MPU_FREQ_HZ_DEFAULT)) {
        prevGyroSysTickMs = currSysTickMs;

        status = mpu_get_gyro_reg(reg, &timestamp);
        if (status) {
            return status;
        }
        status = inv_build_gyro(reg, timestamp);
        if (status) {
            return status;
        }

        status = mpu_get_accel_reg(reg, &timestamp);
        if (status) {
            return status;
        }
        data[0] = reg[0];
        data[1] = reg[1];
        data[2] = reg[2];
        status = inv_build_accel(data, 0, timestamp);
        if (status) {
            return status;
        }

        devMpu9250GyroUpdated = 1;
        mpl_updated = 1;
    }

    if ((currSysTickMs - prevCompassSysTickMs) >= (1000 / COMPASS_FREQ_HZ_DEFAULT)) {
        prevCompassSysTickMs = currSysTickMs;

        status = mpu_get_compass_reg(reg, &timestamp);
        if (status) {
            return status;
        }
        data[0] = reg[0];
        data[1] = reg[1];
        data[2] = reg[2];
        status = inv_build_compass(data, 0, timestamp);
        if (status) {
            return status;
        }

        devMpu9250CompassUpdated = 1;
        mpl_updated = 1;
    }

    if (mpl_updated) {
        status = inv_execute_on_data();
    }

    return status;
}

int devMpu9250GetGyroFloat(float *data, int8_t *accuracy, inv_time_t *timestamp) {
    long q16Data[3];

    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }
    int status = inv_get_sensor_type_gyro(q16Data, accuracy, timestamp);
    if (irqEnabled) {
        ei();
    }

    data[0] = inv_q16_to_float(q16Data[0]);
    data[1] = inv_q16_to_float(q16Data[1]);
    data[2] = inv_q16_to_float(q16Data[2]);

    return status;
}

int devMpu9250GetAccelFloat(float *data, int8_t *accuracy, inv_time_t *timestamp) {
    long q16Data[3];

    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }
    int status = inv_get_sensor_type_accel(q16Data, accuracy, timestamp);
    if (irqEnabled) {
        ei();
    }

    data[0] = inv_q16_to_float(q16Data[0]);
    data[1] = inv_q16_to_float(q16Data[1]);
    data[2] = inv_q16_to_float(q16Data[2]);

    return status;
}

int devMpu9250GetCompassFloat(float *data, int8_t *accuracy, inv_time_t *timestamp) {
    long q16Data[3];

    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }
    int status = inv_get_sensor_type_compass(q16Data, accuracy, timestamp);
    if (irqEnabled) {
        ei();
    }

    data[0] = inv_q16_to_float(q16Data[0]);
    data[1] = inv_q16_to_float(q16Data[1]);
    data[2] = inv_q16_to_float(q16Data[2]);

    return status;
}

int devMpu9250GetQuatFloat(float *data, int8_t *accuracy, inv_time_t *timestamp) {
    long q30Data[4];
    int8_t tmp_ac;
    inv_time_t tmp_ts;

    // API inv_get_sensor_type_quat() did not handle the case accuracy & timestmap is NULL.
    if (accuracy == NULL) {
        accuracy = &tmp_ac;
    }
    if (timestamp == NULL) {
        timestamp = &tmp_ts;
    }

    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }
    int status = inv_get_sensor_type_quat(q30Data, accuracy, timestamp);
    if (irqEnabled) {
        ei();
    }

    data[0] = inv_q30_to_float(q30Data[0]);
    data[1] = inv_q30_to_float(q30Data[1]);
    data[2] = inv_q30_to_float(q30Data[2]);
    data[3] = inv_q30_to_float(q30Data[3]);

    return status;
}

int devMpu9250GetEulerFloat(float *data, int8_t *accuracy, inv_time_t *timestamp) {
    long q16Data[3];
    int8_t tmp_ac;
    inv_time_t tmp_ts;

    // API inv_get_sensor_type_quat() did not handle the case accuracy & timestmap is NULL.
    if (accuracy == NULL) {
        accuracy = &tmp_ac;
    }
    if (timestamp == NULL) {
        timestamp = &tmp_ts;
    }

    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }
    int status = inv_get_sensor_type_euler(q16Data, accuracy, timestamp);
    if (irqEnabled) {
        ei();
    }

    data[0] = inv_q16_to_float(q16Data[0]);
    data[1] = inv_q16_to_float(q16Data[1]);
    data[2] = inv_q16_to_float(q16Data[2]);

    return status;
}

int devMpu9250GetRotMatFloat(float *data, int8_t *accuracy, inv_time_t *timestamp) {
    long q30Data[9];
    int8_t tmp_ac;
    inv_time_t tmp_ts;

    // API inv_get_sensor_type_quat() did not handle the case accuracy & timestmap is NULL.
    if (accuracy == NULL) {
        accuracy = &tmp_ac;
    }
    if (timestamp == NULL) {
        timestamp = &tmp_ts;
    }

    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }
    int status = inv_get_sensor_type_rot_mat(q30Data, accuracy, timestamp);
    if (irqEnabled) {
        ei();
    }

    data[0] = inv_q30_to_float(q30Data[0]);
    data[1] = inv_q30_to_float(q30Data[1]);
    data[2] = inv_q30_to_float(q30Data[2]);
    data[3] = inv_q30_to_float(q30Data[3]);
    data[4] = inv_q30_to_float(q30Data[4]);
    data[5] = inv_q30_to_float(q30Data[5]);
    data[6] = inv_q30_to_float(q30Data[6]);
    data[7] = inv_q30_to_float(q30Data[7]);
    data[8] = inv_q30_to_float(q30Data[8]);

    return status;
}

int devMpu9250GetHeadingFloat(float *data, int8_t *accuracy, inv_time_t *timestamp) {
    long q16Data[1];
    int8_t tmp_ac;
    inv_time_t tmp_ts;

    // API inv_get_sensor_type_quat() did not handle the case accuracy & timestmap is NULL.
    if (accuracy == NULL) {
        accuracy = &tmp_ac;
    }
    if (timestamp == NULL) {
        timestamp = &tmp_ts;
    }

    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }
    int status = inv_get_sensor_type_heading(q16Data, accuracy, timestamp);
    if (irqEnabled) {
        ei();
    }

    data[0] = inv_q16_to_float(q16Data[0]);

    return status;
}
