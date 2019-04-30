import { Injectable } from '@angular/core';
import { Observable, of } from 'rxjs';
import { catchError, map, tap } from 'rxjs/operators';
import { AbstractService } from './abstractService';


@Injectable({
  providedIn: 'root'
})
export class TankControlService extends AbstractService {

  private commandServieUrl = this.baseUrl + '/api/Port/v1/sendCommand';

  sendControl(command: String): Observable<string> {
    return this.http
      .post<string>(this.commandServieUrl, {
        "command":"$AP0:130X0Y!","executionTime":"8000"
      },this.httpOptions)
      .pipe(
        tap(_ => this.log('send command succesfuly ')),
        catchError(err => {
          console.error(err.message);
          console.log("Error is handled");
          return of("Z");
        })
      );
  }

  sendUpCommand(command:String):Observable<string> {
    var speed = 120;
    return this.http
    .post<string>(this.commandServieUrl, [{"command":"$AP0:130X" + (127-speed) + "Y!","executionTime":null}],this.httpOptions)
    .pipe(
      tap(_ => 
        this.log('send up command succesfuly ')),
        catchError(err => {
          console.error(err.message);
          console.log("Error is handled");
          return of("Z");
        })
    );
  }

  sendBackCommand(command:String):Observable<string> {
    var speed = 120;
    return this.http
    .post<string>(this.commandServieUrl, [{
      "command":"$AP0:127X" + (127+speed) + "Y!","executionTime":null
    }],this.httpOptions)
    .pipe(
      tap(_ => this.log('send back command succesfuly ')),
      catchError(this.handleError('sendBackCommand', "failed to send back command"))
    );
  }

  sendLeftCommand(command:String):Observable<string> {
    var speed = 50;
    return this.http
    .post<string>(this.commandServieUrl, [{
      "command":"$AP0:" + (127-speed) + "X127Y!","executionTime":null
    }],this.httpOptions)
    .pipe(
      tap(_ => this.log('send left command succesfuly ')),
      catchError(this.handleError('sendLeftCommand', "failed to send left command"))
    );
  }

  sendRightCommand(command:String):Observable<string> {
    var speed = 50;
    return this.http
    .post<string>(this.commandServieUrl, [{
      "command":"$AP0:" + (127+speed) + "X127Y!","executionTime":null
    }],this.httpOptions)
    .pipe(
      tap(_ => this.log('send right command succesfuly ')),
      catchError(this.handleError('sendRightCommand', "failed to send right command"))
    );
  }

  sendStopCommand(command:String):Observable<string> {
    var speed = 120;
    return this.http
    .post<string>(this.commandServieUrl, [{
      "command":"$AP0:127X127Y!","executionTime":null
    }],this.httpOptions)
    .pipe(
      tap(_ => this.log('send stop command succesfuly ')),
      catchError(this.handleError('sendStopCommand', "failed to send stop command"))
    );

  }


}
