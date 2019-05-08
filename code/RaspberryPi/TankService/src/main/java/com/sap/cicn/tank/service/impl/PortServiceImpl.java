package com.sap.cicn.tank.service.impl;

import org.springframework.beans.factory.InitializingBean;
import org.springframework.stereotype.Service;

import com.fazecast.jSerialComm.SerialPort;
import com.sap.cicn.tank.common.exception.InternalException;
import com.sap.cicn.tank.common.logger.LightLogger;
import com.sap.cicn.tank.service.PortService;

/**
 * Created by i065037 on 2019/3/25.
 */
@Service
public class PortServiceImpl implements PortService, InitializingBean {
    private final LightLogger log = LightLogger.getLogger(this);

    private String comPortDev = "/dev/ttyS0";
    private SerialPort comPort = null;

    /**
     * {@inheritDoc}
     */
    @Override
    public void afterPropertiesSet() throws Exception {
        try {
            comPort = SerialPort.getCommPort(comPortDev);
            comPort.openPort();
            comPort.setComPortParameters(115200, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
            comPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING | SerialPort.TIMEOUT_WRITE_BLOCKING, 500, 500);

            log.info("Open serial port success for tank control board. comPort[", comPortDev, "]");
        } catch (Exception e) {
            log.error(e, "Error open serial port for tank control board. comPort[", comPortDev, "]");
            throw new InternalException(e, "Error open serial port for tank control board. comPort[", comPortDev, "]");
        }
    }

    @Override
    public Boolean sendCommandToPort(String command) {
        try {
            int length = comPort.writeBytes(command.getBytes(), command.length());
            if (length == command.length()) {
                log.info("Send command to tank control board success. command[", command, "]");
            } else {
                log.error("Error send command to tank control board. command[", command, "] expectLength[", command.length(), "] writeLength[", length, "]");
            }
        } catch (Exception e) {
            log.info(e, "Error send command to tank control board. command[", command, "]");
            throw new InternalException(e, "Error send command to tank control board. command[", command, "]");
        }

        return false;
    }

}
