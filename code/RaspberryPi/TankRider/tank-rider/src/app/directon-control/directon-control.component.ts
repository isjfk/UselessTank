import { Component, HostListener,ElementRef,Renderer,HostBinding, OnInit } from '@angular/core';
import { TankControlService } from "./../../service/tankControl.service";
import { HttpClient, HttpHeaders } from '@angular/common/http';

@Component({
  selector: 'app-directon-control',
  templateUrl: './directon-control.component.html',
  //template:``,
  styleUrls: ['./directon-control.component.less']
})
export class DirectonControlComponent implements OnInit {

  constructor(private el: ElementRef,
              private renderer: Renderer) { }

  ngOnInit() {
  }

  //@HostBinding() public pressKey: boolean = false


  @HostListener('window:keydown', ['$event']) 
    pressEvent(event: any){
        if(event.keyCode === 38)  {
          let highlightup: HTMLElement = this.el.nativeElement.querySelector("#upbtn");
          // TankControlService tankControl = new TankControlService();
          // sendControl("$AP0:130X0Y!","8000");
          highlightup.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightup.style.backgroundColor = 'white';
          }, 1 * 1000);
        }

        else if(event.keyCode === 40){
          let highlightdown: HTMLElement = this.el.nativeElement.querySelector("#downbtn");
          highlightdown.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightdown.style.backgroundColor = 'white';
          }, 1 * 1000);
        }

        else if(event.keyCode === 37){
          let highlightleft: HTMLElement = this.el.nativeElement.querySelector("#leftbtn");
          highlightleft.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightleft.style.backgroundColor = 'white';
          }, 1 * 1000);
        }

        else if(event.keyCode === 39){
          let highlightright: HTMLElement = this.el.nativeElement.querySelector("#rightbtn");
          highlightright.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightright.style.backgroundColor = 'white';
          }, 1 * 1000);
        }
  }

}
