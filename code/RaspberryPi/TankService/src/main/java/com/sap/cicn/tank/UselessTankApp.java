package com.sap.cicn.tank;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.annotation.Bean;

import com.sap.cicn.tank.spring.ApplicationContextHolder;


/**
 * Created by i065037 on 2019/3/25.
 */
@SpringBootApplication
public class UselessTankApp {

    public static void main(String[] args) {
        SpringApplication.run(UselessTankApp.class,args);
    }

    @Bean
    public ApplicationContextHolder applicationContextHolder() {
        return new ApplicationContextHolder();
    }

}
