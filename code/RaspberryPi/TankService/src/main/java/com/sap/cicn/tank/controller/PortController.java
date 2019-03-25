package com.sap.cicn.tank.controller;

import com.sap.cicn.tank.api.domain.CommandRequestParam;
import com.sap.cicn.tank.service.PortService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;

/**
 * Created by i065037 on 2019/3/25.
 */
@Controller
@RequestMapping(value = {"/api/Port/v1"})
public class PortController {

    @Autowired
    private PortService portService;

    @RequestMapping(value = "/sendCommand", method = { RequestMethod.POST })
    public ResponseEntity<Boolean> sendSerialCommand(@RequestBody CommandRequestParam param){
        Boolean result = portService.sendCommandToPort(param.getCommand());
        return  new ResponseEntity<Boolean>(result, HttpStatus.OK);
    }

}
