/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.hardware.driver;

import java.math.BigDecimal;
import java.util.HashMap;
import java.util.Map;

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.sap.cicn.tank.common.exception.InternalException;
import com.sap.cicn.tank.common.logger.LightLogger;
import com.sap.cicn.tank.hardware.i2c.I2CDeviceFactory;
import com.sap.cicn.tank.hardware.i2c.I2cAddress;
import com.sap.cicn.tank.util.SystemUtils;

/**
 * @author I311334
 */
public class PCA9635Driver {

    private static LightLogger log = LightLogger.getLogger(PCA9635Driver.class);

    public static final int I2C_BUS_NUMBER_DEFAULT = I2CBus.BUS_1;
    public static final int I2C_ADDRESS_DEFAULT = 0x40;

    public static final int MODE1_ADDRESS = 0x00;
    public static final int MODE2_ADDRESS = 0x01;
    public static final int PRESCALE_ADDRESS = 0xFE;
    public static final int CHANNEL0_ADDRESS = 0x06;

    public static final int MODE1_BIT_RESTART = 0x80;
    public static final int MODE1_BIT_SLEEP = 0x10;
    public static final int MODE2_BIT_TOTEM_POLE = 0x04;

    public static final int CHANNEL_LENGTH = 4;
    public static final int ON_L_OFFSET = 0;
    public static final int ON_H_OFFSET = 1;
    public static final int OFF_L_OFFSET = 2;
    public static final int OFF_H_OFFSET = 3;

    private static Map<I2cAddress, PCA9635Driver> driverMap = new HashMap<>();

    private I2cAddress address;
    private I2CDevice device;

    private long oscClock = 25000000;       // 25MHz
    private long pwmFreq = 50;              // 50Hz

    private PCA9635Driver(I2cAddress address, I2CDevice device) {
        super();
        this.address = address;
        this.device = device;
    }

    public static synchronized PCA9635Driver getInstance() {
        return getInstance(I2C_BUS_NUMBER_DEFAULT, I2C_ADDRESS_DEFAULT);
    }

    public static synchronized PCA9635Driver getInstance(int i2cAddress) {
        return getInstance(I2C_BUS_NUMBER_DEFAULT, i2cAddress);
    }

    public static synchronized PCA9635Driver getInstance(int i2cBusNumber, int i2cAddress) {
        I2cAddress address = new I2cAddress(i2cBusNumber, i2cAddress);

        PCA9635Driver driver = driverMap.get(address);
        if (driver == null) {
            driver = new PCA9635Driver(address, I2CDeviceFactory.getI2cDevice(i2cBusNumber, i2cAddress));

            driverMap.put(address, driver);
        }

        return driver;
    }

    public PCA9635Driver oscClock(long oscClock) {
        this.oscClock = oscClock;
        return this;
    }

    public PCA9635Driver pwmFreq(long pwmFreq) {
        this.pwmFreq = pwmFreq;
        return this;
    }

    public synchronized PCA9635Driver init() {
        try {
            int mode1 = device.read(MODE1_ADDRESS);

            // Enter sleep mode before change parameters.
            mode1 &= ~MODE1_BIT_RESTART;
            mode1 |= MODE1_BIT_SLEEP;
            device.write(MODE1_ADDRESS, (byte) mode1);

            BigDecimal preScale = BigDecimal.valueOf(oscClock);
            preScale = preScale.divide(BigDecimal.valueOf(4096), BigDecimal.ROUND_HALF_UP);
            preScale = preScale.divide(BigDecimal.valueOf(pwmFreq), BigDecimal.ROUND_HALF_UP);
            preScale = preScale.setScale(0, BigDecimal.ROUND_HALF_UP);
            preScale = preScale.subtract(BigDecimal.valueOf(1));

            // Change parameters.
            device.write(PRESCALE_ADDRESS, preScale.byteValue());
            device.write(MODE2_ADDRESS, (byte) 0x04);

            // Exit sleep mode.
            mode1 &= ~MODE1_BIT_RESTART;
            mode1 &= ~MODE1_BIT_SLEEP;
            device.write(MODE1_ADDRESS, (byte) mode1);
            SystemUtils.sleepAtLeast(200);

            // Restart PWM channel.
            mode1 |= MODE1_BIT_RESTART;
            device.write(MODE1_ADDRESS, (byte) mode1);
        } catch (Exception e) {
            log.error(e, "Error initialize PCA9635 on I2C bus[", address.getBusNumber(), "] address[", address.getAddress(), "].");
            throw new InternalException(e, "Error initialize PCA9635 on I2C bus[", address.getBusNumber(), "] address[", address.getAddress(), "].");
        }

        return this;
    }

    public synchronized void close() {
        try {
            int mode1 = device.read(MODE1_ADDRESS);
            mode1 &= ~MODE1_BIT_RESTART;        // Don't update restart status.
            mode1 |= MODE1_BIT_SLEEP;           // Enter sleep mode.

            device.write(MODE1_ADDRESS, (byte) mode1);
        } catch (Exception e) {
            log.error(e, "Error close PCA9635 on I2C bus[", address.getBusNumber(), "] address[", address.getAddress(), "].");
            throw new InternalException(e, "Error close PCA9635 on I2C bus[", address.getBusNumber(), "] address[", address.getAddress(), "].");
        }
    }

    public synchronized void setPwm(byte channel, short onValue, short offValue) {
        try {
            device.write(CHANNEL0_ADDRESS + (channel * CHANNEL_LENGTH) + ON_L_OFFSET, (byte) onValue);
            device.write(CHANNEL0_ADDRESS + (channel * CHANNEL_LENGTH) + ON_H_OFFSET, (byte) (onValue >> 8));
            device.write(CHANNEL0_ADDRESS + (channel * CHANNEL_LENGTH) + OFF_L_OFFSET, (byte) offValue);
            device.write(CHANNEL0_ADDRESS + (channel * CHANNEL_LENGTH) + OFF_H_OFFSET, (byte) (offValue >> 8));
        } catch (Exception e) {
            log.error(e, "Error write to PCA9635 on I2C bus[", address.getBusNumber(), "] address[", address.getAddress(), "] channel[", channel, "] onValue[", onValue, "] offValue[", offValue, "].");
            throw new InternalException(e, "Error write to PCA9635 on I2C bus[", address.getBusNumber(), "] address[", address.getAddress(), "] channel[", channel, "] onValue[", onValue, "] offValue[", offValue, "].");
        }
    }

    /**
     * Set the servo pwm pulse time.
     *
     * @param channel PCA9635 channel
     * @param pulseValue servo pulse time in us
     */
    public void setServoPulse(byte channel, int pulseValue) {
        int pwmOffValue = pulseValue * 4096 / 20000;
        setPwm(channel, (short) 0, (short) pwmOffValue);
    }

    public void setServoAngle(byte channel, BigDecimal angle) {
        int pulseValue = angle.multiply(BigDecimal.valueOf(2000)).divide(BigDecimal.valueOf(180), BigDecimal.ROUND_HALF_UP).add(BigDecimal.valueOf(500)).intValue();
        setServoPulse(channel, pulseValue);
    }

}
