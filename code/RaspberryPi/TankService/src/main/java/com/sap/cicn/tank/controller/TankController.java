package com.sap.cicn.tank.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;

import com.sap.cicn.tank.api.domain.TankControl;
import com.sap.cicn.tank.api.domain.TankControlRange;
import com.sap.cicn.tank.service.TankControlService;

@Controller
@RequestMapping(value = {"api/TankControl/v1"})
public class TankController {

    @Autowired
    private TankControlService tankControlService;

    @RequestMapping(method = { RequestMethod.GET })
    public ResponseEntity<TankControl> get(){
        TankControl tankControl = tankControlService.getTankControl();
        return new ResponseEntity<TankControl>(tankControl, HttpStatus.OK);
    }

	@RequestMapping(method = { RequestMethod.POST })
	public ResponseEntity<TankControl> post(@RequestBody TankControl control){
		TankControl tankControl = tankControlService.setTankControl(control);
		return new ResponseEntity<TankControl>(tankControl, HttpStatus.OK);
	}

    @RequestMapping(value="/range", method = { RequestMethod.GET })
    public ResponseEntity<TankControlRange> getRange(){
        TankControlRange tankControlRange = tankControlService.getTankControlRange();
        return new ResponseEntity<TankControlRange>(tankControlRange, HttpStatus.OK);
    }

    /**
     * @return the tankControlService
     */
    public TankControlService getTankControlService() {
        return tankControlService;
    }

    /**
     * @param tankControlService the tankControlService to set
     */
    public void setTankControlService(TankControlService tankControlService) {
        this.tankControlService = tankControlService;
    }

}