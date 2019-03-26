package com.sap.cicn.tank.task;

import com.sap.cicn.tank.api.domain.Command;

import java.util.LinkedList;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Created by i065037 on 2019/3/26.
 */
public class CommandContainer {
    private static BlockingQueue<Command> commandQueue = new LinkedBlockingQueue<>();

    public static BlockingQueue<Command> getCommandQueue() {
        return commandQueue;
    }

    public static void setCommandQueue(BlockingQueue<Command> commandQueue) {
        CommandContainer.commandQueue = commandQueue;
    }

    public static void addCommand(Command command)  {
        try {
            commandQueue.put(command);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public  static void removeCommand(Command command){
        commandQueue.remove(command);
    }
}
