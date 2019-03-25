package com.sap.cicn.tank.service.impl;

import com.fazecast.jSerialComm.SerialPort;
import com.sap.cicn.tank.service.PortService;
import org.springframework.stereotype.Service;

/**
 * Created by i065037 on 2019/3/25.
 */
@Service
public class PortServiceImpl implements PortService {
    @Override
    public Boolean sendCommandToPort(String command) {
        Boolean result =false;
        SerialPort comPort = SerialPort.getCommPorts()[4];
        comPort.openPort();
        try {
            while (true)
            {
//                while (comPort.bytesAvailable() == 0)
//                    Thread.sleep(20);
               // comPort.setComPortParameters(9600,8,1,0);
                comPort.setComPortParameters(9600,8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);

                comPort.writeBytes(command.getBytes(), command.length());
                byte[] readBuffer = new byte[comPort.bytesAvailable()];
                int numRead = comPort.readBytes(readBuffer, readBuffer.length);
                if(numRead != 0) result = true;
                System.out.println("Read " + numRead + " bytes.");
                break;
            }
        } catch (Exception e) { e.printStackTrace(); }
        comPort.closePort();
        return result;
    }
}
