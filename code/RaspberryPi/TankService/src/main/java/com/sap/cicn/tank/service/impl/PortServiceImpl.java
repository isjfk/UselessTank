package com.sap.cicn.tank.service.impl;

import com.fazecast.jSerialComm.SerialPort;
import com.sap.cicn.tank.common.logger.LightLogger;
import com.sap.cicn.tank.service.PortService;
import org.springframework.stereotype.Service;

/**
 * Created by i065037 on 2019/3/25.
 */
@Service
public class PortServiceImpl implements PortService {
    private final LightLogger LOGGER = LightLogger.getLogger(this);

    private SerialPort comPort = null;

    public void init() {
        if (comPort != null) {
            return;
        }
        comPort = SerialPort.getCommPort("/dev/ttyS0");
        comPort.openPort();
        comPort.setComPortParameters(115200, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
        comPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING | SerialPort.TIMEOUT_WRITE_BLOCKING, 200, 200);
    }

    @Override
    public Boolean sendCommandToPort(String command) {
        this.init();
        // SerialPort comPort = SerialPort.getCommPorts()[4];

        try {
            int length = comPort.writeBytes(command.getBytes(), command.length());
            return length == command.length();
        } catch (Exception e) {
            LOGGER.error("failed to execute command", e);
            try {
                comPort.closePort();
            } catch (Exception ex) {

            }
            comPort = null;
        }

        return false;
    }
}
