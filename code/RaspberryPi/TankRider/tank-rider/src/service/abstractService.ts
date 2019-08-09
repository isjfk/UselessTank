import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Observable, of } from 'rxjs';
import config from "../../config.json";
import { Location } from '@angular/common';

@Injectable({
    providedIn: 'root'
})
export abstract class AbstractService {

    //protected baseUrl = config.tank_service_url;  
    protected baseUrl = "http://" + location.hostname + ":8080";

    protected httpOptions = {
        headers: new HttpHeaders({
            'Content-Type': 'application/json',
            'Authorization': 'my-auth-token'
        })
    };

    constructor(protected http: HttpClient) {
    }

    protected log(message: string) {
        console.log(message);
    }

    protected handleError<T>(operation = 'operation', result?: T) {
        return (error: any): Observable<T> => {
            console.log(error);
            return of(result as T);
        };
    }
}