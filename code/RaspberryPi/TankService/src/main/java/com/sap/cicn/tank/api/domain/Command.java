package com.sap.cicn.tank.api.domain;

/**
 * Created by i065037 on 2019/3/25.
 */
public class Command {

    private String command;
    private Long executionTime;

    public String getCommand() {
        return command;
    }

    public void setCommand(String command) {
        this.command = command;
    }

    public Long getExecutionTime() {
        return executionTime;
    }

    public void setExecutionTime(Long executionTime) {
        this.executionTime = executionTime;
    }

    @Override
    public String toString() {
        return "Command{" +
                "command='" + command + '\'' +
                ", executionTime=" + executionTime +
                '}';
    }
}
