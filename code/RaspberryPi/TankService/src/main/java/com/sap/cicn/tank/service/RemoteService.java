package com.sap.cicn.tank.service;

import org.springframework.stereotype.Service;

@Service
public interface RemoteService {
	Boolean goForward();
	Boolean goBack();
	Boolean turnLeft();
	Boolean turnRight();
	Boolean stop();
	Boolean setSpeed(double speed);
	Boolean walkADistance(double distance);
	Boolean turnDirection(String direction);
	Boolean setRotateAngularVelocity(double anglVelocity);
	Boolean setRotateAngularVelocityOffset(double offset);
}