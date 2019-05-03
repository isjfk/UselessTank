/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.hardware.i2c;

import java.util.HashMap;
import java.util.Map;

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;
import com.sap.cicn.tank.common.exception.InternalException;
import com.sap.cicn.tank.common.logger.LightLogger;

/**
 * @author I311334
 */
public class I2CDeviceFactory {

    private static final LightLogger log = LightLogger.getLogger(I2CDeviceFactory.class);

    private static Map<Integer, I2CBus> i2cBusMap = new HashMap<>();

    public static synchronized I2CDevice getI2cDevice(int busNumber, int address) {
        try {
            I2CBus bus = i2cBusMap.get(busNumber);
            if (bus == null) {
                bus = I2CFactory.getInstance(busNumber);
                i2cBusMap.put(busNumber, bus);
            }

            return bus.getDevice(address);
        } catch (Exception e) {
            log.error(e, "Error initialize I2C device on bus[", busNumber, "] address[", address, "].");
            throw new InternalException(e, "Error initialize I2C device on bus[", busNumber, "] address[", address, "].");
        }
    }

    public static synchronized void close() {
        for (I2CBus bus : i2cBusMap.values()) {
            try {
                bus.close();
            } catch (Exception e) {
                log.warn(e, "Error close I2C bus[", bus.getBusNumber(), "].");
            }
        }

        i2cBusMap.clear();
    }

}
