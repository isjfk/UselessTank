package com.sap.cicn.tank.service.impl;

import com.fazecast.jSerialComm.SerialPort;
import com.sap.cicn.tank.logger.LightLogger;
import com.sap.cicn.tank.service.PortService;
import org.springframework.stereotype.Service;

/**
 * Created by i065037 on 2019/3/25.
 */
@Service
public class PortServiceImpl implements PortService {
    private final LightLogger LOGGER = LightLogger.getLogger(this);

    @Override
    public Boolean sendCommandToPort(String command) {
        Boolean result =false;
       // SerialPort comPort = SerialPort.getCommPorts()[4];
        SerialPort comPort = SerialPort.getCommPort("/dev/ttyS0");
        comPort.openPort();
        try {
            while (true)
            {
                comPort.setComPortParameters(115200,8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
                comPort.writeBytes(command.getBytes(), command.length());
                byte[] readBuffer = new byte[comPort.bytesAvailable()];
                int numRead = comPort.readBytes(readBuffer, readBuffer.length);
                if(numRead != 0) result = true;
                LOGGER.info("Read " + numRead + " bytes.");
                break;
            }
        } catch (Exception e) { LOGGER.error("failed to execute command", e);}
        comPort.closePort();
        return result;
    }
}
