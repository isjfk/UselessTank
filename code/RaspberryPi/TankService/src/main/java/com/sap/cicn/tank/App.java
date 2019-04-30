package com.sap.cicn.tank;

import com.sap.cicn.tank.spring.ApplicationContextHolder;
import com.sap.cicn.tank.task.CommandExecutor;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.annotation.Bean;


/**
 * Created by i065037 on 2019/3/25.
 */
@SpringBootApplication
public class App {

    public static void main(String[] args) {
        SpringApplication.run(App.class,args);
       // Thread commandThread = new Thread(new CommandExecutor());
        //commandThread.start();
    }

    @Bean
    public ApplicationContextHolder applicationContextHolder() {
        return new ApplicationContextHolder();
    }

}
