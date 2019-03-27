package com.sap.cicn.tank.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;

import com.sap.cicn.tank.service.RemoteService;

@Controller
@RequestMapping(value = {"api/remote/v1"})
public class RemoteController {
	@Autowired RemoteService remoteService;
	
	@RequestMapping(value = "/forward", method = { RequestMethod.POST })
	public ResponseEntity<Boolean> goForward(){
		Boolean result= remoteService.goForward();
		return new ResponseEntity<Boolean>(result, HttpStatus.OK);
	}
	
	@RequestMapping(value = "/back", method = { RequestMethod.POST })
	public ResponseEntity<Boolean> goBack(){
		Boolean result= remoteService.goBack();
		return new ResponseEntity<Boolean>(result, HttpStatus.OK);
	}
	
	@RequestMapping(value = "/left", method = { RequestMethod.POST })
	public ResponseEntity<Boolean> turnLeft(){
		Boolean result= remoteService.turnLeft();
		return new ResponseEntity<Boolean>(result, HttpStatus.OK);
	}
	
	@RequestMapping(value = "/right", method = { RequestMethod.POST })
	public ResponseEntity<Boolean> turnRight(){
		Boolean result= remoteService.turnRight();
		return new ResponseEntity<Boolean>(result, HttpStatus.OK);
	}
	
	@RequestMapping(value = "/stop", method = { RequestMethod.POST })
	public ResponseEntity<Boolean> stop(){
		Boolean result= remoteService.stop();
		return new ResponseEntity<Boolean>(result, HttpStatus.OK);
	}
	
	@RequestMapping(value = "/speed/{speed}", method = { RequestMethod.POST })
	public ResponseEntity<Boolean> setSpeed(@PathVariable("speed") Double speed){
		Boolean result= remoteService.setSpeed(speed);
		return new ResponseEntity<Boolean>(result, HttpStatus.OK);
	}
	
	@RequestMapping(value = "/distance/{distance}", method = { RequestMethod.POST })
	public ResponseEntity<Boolean> walkADistance(@PathVariable("distance") Double distance){
		Boolean result= remoteService.walkADistance(distance);
		return new ResponseEntity<Boolean>(result, HttpStatus.OK);
	}
	
	@RequestMapping(value = "/direction/{direction}", method = { RequestMethod.POST })
	public ResponseEntity<Boolean> turnDirection(@PathVariable("direction") String direction){
		Boolean result= remoteService.turnDirection(direction);
		return new ResponseEntity<Boolean>(result, HttpStatus.OK);
	}
	
	@RequestMapping(value = "/angleVelocity/{angleVelocity}", method = { RequestMethod.POST })
	public ResponseEntity<Boolean> setRotateAngularVelocity(@PathVariable("angleVelocity") Double angleVelocity){
		Boolean result= remoteService.setRotateAngularVelocity(angleVelocity);
		return new ResponseEntity<Boolean>(result, HttpStatus.OK);
	}
	
	@RequestMapping(value = "/angleVelocityOffset/{offset}", method = { RequestMethod.POST })
	public ResponseEntity<Boolean> setRotateAngularVelocityOffset(@PathVariable("offset") Double offset){
		Boolean result= remoteService.setRotateAngularVelocityOffset(offset);
		return new ResponseEntity<Boolean>(result, HttpStatus.OK);
	}
}