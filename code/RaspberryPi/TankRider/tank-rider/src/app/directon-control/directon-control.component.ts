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

  //@HostBinding() public pressKey: boolean = false


  @HostListener('window:keydown', ['$event']) 
    pressEvent(event: any){
        if(event.keyCode === 87)  {
          let highlightup: HTMLElement = this.el.nativeElement.querySelector("#upbtn");
          this.tankcontrol.sendUpCommand("forward").subscribe(result=>{});
          //this.tankcontrol.sendControl("http://10.128.167.223:8080/api/Port/v1/sendCommand");
          highlightup.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightup.style.backgroundColor = 'white';
          }, 1 * 1000);
        }

        else if(event.keyCode === 83){
          let highlightdown: HTMLElement = this.el.nativeElement.querySelector("#downbtn");
          //this.tankcontrol.sendControl("http://10.128.167.223:8080/api/Port/v1/sendCommand");
          this.tankcontrol.sendBackCommand("back").subscribe(result=>{});
          highlightdown.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightdown.style.backgroundColor = 'white';
          }, 1 * 1000);
        }

        else if(event.keyCode === 65){
          let highlightleft: HTMLElement = this.el.nativeElement.querySelector("#leftbtn");
          //this.tankcontrol.sendControl("http://10.128.167.223:8080/api/Port/v1/sendCommand");
          this.tankcontrol.sendLeftCommand("left").subscribe(result=>{});
          highlightleft.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightleft.style.backgroundColor = 'white';
          }, 1 * 1000);
        }

        else if(event.keyCode === 68){
          let highlightright: HTMLElement = this.el.nativeElement.querySelector("#rightbtn");
          //this.tankcontrol.sendControl("http://10.128.167.223:8080/api/Port/v1/sendCommand");
          this.tankcontrol.sendRightCommand("right").subscribe(result=>{});
          highlightright.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightright.style.backgroundColor = 'white';
          }, 1 * 1000);
        }

        else if(event.keyCode === 32){
          let highlightright: HTMLElement = this.el.nativeElement.querySelector("#cyclebtn");
          //this.tankcontrol.sendControl("http://10.128.167.223:8080/api/Port/v1/sendCommand");
          this.tankcontrol.sendStopCommand("stop").subscribe(result=>{}); 
          highlightright.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightright.style.backgroundColor = 'white';
          }, 1 * 1000);
        }
  }

}
