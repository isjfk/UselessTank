package com.sap.cicn.tank.api.domain;

/**
 * Created by i065037 on 2019/3/25.
 */
public class CommandRequestParam {
    private String command;

    public String getCommand() {
        return command;
    }

    public void setCommand(String command) {
        this.command = command;
    }

    @Override
    public String toString() {
        return "CommandRequestParam{" +
                "command='" + command + '\'' +
                '}';
    }
}
