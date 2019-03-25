package com.sap.cicn.tank;

import com.sap.cicn.tank.service.PortService;
import com.sap.cicn.tank.service.impl.PortServiceImpl;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

/**
 * Created by i065037 on 2019/3/25.
 */
@SpringBootApplication
public class App {

    public static void main(String[] args) {
        SpringApplication.run(App.class,args);
    }
}
