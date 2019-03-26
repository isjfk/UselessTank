package com.sap.cicn.tank.task;

import com.sap.cicn.tank.api.domain.Command;
import com.sap.cicn.tank.logger.LightLogger;
import com.sap.cicn.tank.service.PortService;
import com.sap.cicn.tank.spring.ApplicationContextHolder;

/**
 * Created by i065037 on 2019/3/26.
 */
public class CommandExecutor implements Runnable{

    private final LightLogger LOGGER = LightLogger.getLogger(this);
    private PortService portService = ApplicationContextHolder.getBean(PortService.class);

    @Override
    public void run() {
        LOGGER.info("Command executor start");
        while(true){
            Command command = CommandContainer.getCommandQueue().poll();
            if(command==null){
                //LOGGER.info("Current no command in the queue");
                continue;
            }
            Boolean result = portService.sendCommandToPort(command.getCommand());
            try {
                if(command.getExecutionTime()!=null) {
                    Thread.sleep(command.getExecutionTime());
                }
            } catch (InterruptedException e) {
               LOGGER.error("Failed to execute command", e);
            }
            LOGGER.info("Command executed " + result +" with [ "+ command+" ]");
        }
    }
}
