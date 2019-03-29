import { Injectable } from '@angular/core';
import { Observable, of } from 'rxjs';
import { catchError, map, tap } from 'rxjs/operators';
import { AbstractService } from './abstractService';


@Injectable({
  providedIn: 'root'
})
export class TankControlService extends AbstractService {

  private commandServieUrl = this.baseUrl + 'api/Port/v1/sendCommand';

  sendControl(command: String): Observable<string> {
    return this.http
      .post<string>(this.commandServieUrl, {
        "command":"$AP0:130X0Y!","executionTime":"8000"
      },this.httpOptions)
      .pipe(
        tap(_ => this.log('send command succesfuly ')),
        catchError(this.handleError('sendControl', "failed to send command"))
      );
  }


}
