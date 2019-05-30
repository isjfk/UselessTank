import { Injectable } from '@angular/core';
import { Observable, of } from 'rxjs';
import { catchError, map, tap } from 'rxjs/operators';
import { AbstractService } from './abstractService';


@Injectable({
  providedIn: 'root'
})
export class TankControlService extends AbstractService {

	private commandServieUrl = this.baseUrl + '/api/Port/v1/sendCommand';
    private tankControlServieUrl = this.baseUrl + '/api/TankControl/v1';
	private panTiltServiceUrl = this.baseUrl + '/api/CameraPanTilt/v1';

	sendControlCommand(x:string, y:string): Observable<string> {
		return this.http
			.post<string>(this.commandServieUrl, [{
				"command":"$AP0:" + x + "X" + y + "Y!","executionTime":null
			}], this.httpOptions)
			.pipe(
				tap(_ => this.log('send control command succesfuly')),
				catchError(this.handleError('sendControlCommand', "failed to send control command"))
			);
	}

	sendTankControl(throttle:string, yaw:string): Observable<string> {
		return this.http
			.post<string>(this.tankControlServieUrl, {
				"throttle": throttle,
                "yaw": yaw
			}, this.httpOptions)
			.pipe(
				tap(_ => this.log('send tank control succesfuly')),
				catchError(this.handleError('sendTankControl', "failed to send tank control"))
			);
	}

	sendCameraAngle(pan:string, tilt:string): Observable<string> {
		return this.http
			.post<string>(this.panTiltServiceUrl, {
				"pan":pan,"tilt":tilt
			}, this.httpOptions)
			.pipe(
				tap(_ => this.log('send camera angle succesfuly')),
				catchError(this.handleError('sendCameraAngle', "failed to send camera angle"))
			);
	}

}
