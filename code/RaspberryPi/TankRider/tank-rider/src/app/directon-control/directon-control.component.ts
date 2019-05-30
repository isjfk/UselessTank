import { Component, HostListener,ElementRef,Renderer,HostBinding, OnInit } from '@angular/core';
import { TankControlService } from "./../../service/tankControl.service";

@Component({
  selector: 'app-directon-control',
  templateUrl: './directon-control.component.html',
  //template:``,
  styleUrls: ['./directon-control.component.less']
})
export class DirectonControlComponent implements OnInit {

	constructor(private el: ElementRef,
				private renderer: Renderer,
				private tankcontrol: TankControlService) { }

	ngOnInit() {
	}

	private keyMap: Map<number, boolean> = new Map<number, boolean>();

	@HostListener('window:keydown', ['$event'])
	keydownEvent(event: any){
        //console.log(event.keyCode + " " + event.type);
		if ((event.type == "keydown") && this.keyMap.get(event.keyCode)) {
			return;
		}

		this.keyMap.set(event.keyCode, event.type == "keydown");

		// Space
		if (event.keyCode === 32){
          this.keyMap.clear();
        }

		//console.log(this.keyMap);
		this.sendTankControl();
		this.sendCameraAngle();
	}

	@HostListener('window:keyup',['$event'])
	keyupEvent(event:any){
		//console.log(event.keyCode + " " + event.type);
		this.keyMap.set(event.keyCode, event.type == "keydown");

		// Space
		if (event.keyCode === 32){
          this.keyMap.clear();
        }

		//console.log(this.keyMap);
		this.sendTankControl();
		this.sendCameraAngle();
    }

	@HostListener('window:blur',['$event'])
	focusOutEvent(event:any){
		//console.log(event);
		this.keyMap.clear();

		//console.log(this.keyMap);
		this.sendTankControl();
		this.sendCameraAngle();
    }

	sendControlCommand() {
		let speedValue = 127;
		let turnValue = 60;
		let forwardTurnOffset = 0;
		let backwardTurnOffset = 3;

		let x = 127;
		let y = 127;

		// W
		if (this.keyMap.get(87)) {
			y = y - speedValue;
			x = x + forwardTurnOffset;
			this.el.nativeElement.querySelector("#upbtn").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#upbtn").style.backgroundColor = 'white';
		}

		// S
		if (this.keyMap.get(83)) {
			y = y + speedValue;
			x = x + backwardTurnOffset;
			this.el.nativeElement.querySelector("#downbtn").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#downbtn").style.backgroundColor = 'white';
		}

		// A
		if (this.keyMap.get(65)) {
			x = x - turnValue;
			this.el.nativeElement.querySelector("#leftbtn").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#leftbtn").style.backgroundColor = 'white';
		}

		// D
		if (this.keyMap.get(68)) {
			x = x + turnValue;
			this.el.nativeElement.querySelector("#rightbtn").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#rightbtn").style.backgroundColor = 'white';
		}

		// Fix speed & direction range.
		x = (x < 0) ? 0 : x;
		x = (x > 255) ? 255 : x;
		y = (y < 0) ? 0 : y;
		y = (y > 255) ? 255 : y;

		this.tankcontrol.sendControlCommand(x.toString(), y.toString()).subscribe(result=>{});
	}

	sendTankControl() {
		let throttleValue = 100;
		let yawValue = 30;

		let throttle = 0;
		let yaw = 0;

		// W
		if (this.keyMap.get(87)) {
			throttle = throttle + throttleValue;
			this.el.nativeElement.querySelector("#upbtn").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#upbtn").style.backgroundColor = 'white';
		}

		// S
		if (this.keyMap.get(83)) {
			throttle = throttle - throttleValue;
			this.el.nativeElement.querySelector("#downbtn").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#downbtn").style.backgroundColor = 'white';
		}

		// A
		if (this.keyMap.get(65)) {
			yaw = yaw + yawValue;
			this.el.nativeElement.querySelector("#leftbtn").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#leftbtn").style.backgroundColor = 'white';
		}

		// D
		if (this.keyMap.get(68)) {
			yaw = yaw - yawValue;
			this.el.nativeElement.querySelector("#rightbtn").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#rightbtn").style.backgroundColor = 'white';
		}

		this.tankcontrol.sendTankControl(throttle.toString(), yaw.toString()).subscribe(result=>{});
	}

	sendCameraAngle() {
		let panValue = 70;
		let tiltValue = 50;

		let pan = 0;
		let tilt = 0;

		// UP
		if (this.keyMap.get(38)) {
			tilt = tilt + tiltValue;
			this.el.nativeElement.querySelector("#upbtnForTilt").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#upbtnForTilt").style.backgroundColor = 'white';
		}

		// DOWN
		if (this.keyMap.get(40)) {
			tilt = tilt - tiltValue;
			this.el.nativeElement.querySelector("#downbtnForTilt").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#downbtnForTilt").style.backgroundColor = 'white';
		}

		// LEFT
		if (this.keyMap.get(37)) {
			pan = pan + panValue;
			this.el.nativeElement.querySelector("#leftbtnForPan").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#leftbtnForPan").style.backgroundColor = 'white';
		}

		// RIGHT
		if (this.keyMap.get(39)) {
			pan = pan - panValue;
			this.el.nativeElement.querySelector("#rightbtnForPan").style.backgroundColor = 'yellow';
		} else {
			this.el.nativeElement.querySelector("#rightbtnForPan").style.backgroundColor = 'white';
		}

		this.tankcontrol.sendCameraAngle(pan.toString(), tilt.toString()).subscribe(result=>{});
	}

}
