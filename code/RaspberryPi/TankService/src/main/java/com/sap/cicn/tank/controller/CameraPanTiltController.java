/**
 * Copyright (C) 1972-2019 SAP Co., Ltd. All rights reserved.
 */
package com.sap.cicn.tank.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;

import com.sap.cicn.tank.api.domain.CameraAngle;
import com.sap.cicn.tank.api.domain.CameraAngleRange;
import com.sap.cicn.tank.service.CameraPanTiltService;

/**
 * @author I311334
 */
@Controller
@RequestMapping(value = {"/api/CameraPanTilt/v1"})
public class CameraPanTiltController {

    @Autowired
    private CameraPanTiltService cameraPanTiltService;

    @RequestMapping(method = { RequestMethod.GET })
    public ResponseEntity<CameraAngle> get(){
        CameraAngle angle = cameraPanTiltService.getCameraAngle();
        return new ResponseEntity<>(angle, HttpStatus.OK);
    }

    @RequestMapping(method = { RequestMethod.POST })
    public ResponseEntity<CameraAngle> post(@RequestBody CameraAngle angle){
        CameraAngle newAngle = cameraPanTiltService.setCameraAngle(angle);
        return new ResponseEntity<>(newAngle, HttpStatus.OK);
    }

    @RequestMapping(value = "/range", method = { RequestMethod.GET })
    public ResponseEntity<CameraAngleRange> getRange(){
        CameraAngleRange range = cameraPanTiltService.getCameraAngleRange();
        return new ResponseEntity<>(range, HttpStatus.OK);
    }

    /**
     * @return the cameraPanTiltService
     */
    public CameraPanTiltService getCameraPanTiltService() {
        return cameraPanTiltService;
    }

    /**
     * @param cameraPanTiltService the cameraPanTiltService to set
     */
    public void setCameraPanTiltService(CameraPanTiltService cameraPanTiltService) {
        this.cameraPanTiltService = cameraPanTiltService;
    }

}
