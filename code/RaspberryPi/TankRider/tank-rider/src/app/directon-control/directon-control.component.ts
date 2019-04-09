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

/*
command format $AP0:127X127Y!
X larger than 127 turns right, less than 127 turns left
Y larger than 127 goes forward, less than 127 goes back
*/

  @HostListener('window:keydown', ['$event']) 
    pressEvent(event: any){
        if(event.keyCode === 38)  {
          let highlightup: HTMLElement = this.el.nativeElement.querySelector("#upbtn");
          this.tankcontrol.sendControl(JSON.stringify({
            "command":"$AP0:127X130Y!",
            "executionTime":"8000"
        }));
          highlightup.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightup.style.backgroundColor = 'white';
          }, 1 * 1000);
        }

        else if(event.keyCode === 40){
          let highlightdown: HTMLElement = this.el.nativeElement.querySelector("#downbtn");
          this.tankcontrol.sendControl(JSON.stringify({
            "command":"$AP0:127X67Y!",
            "executionTime":"8000"
          }));
          highlightdown.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightdown.style.backgroundColor = 'white';
          }, 1 * 1000);
        }

        else if(event.keyCode === 37){
          let highlightleft: HTMLElement = this.el.nativeElement.querySelector("#leftbtn");
          this.tankcontrol.sendControl(JSON.stringify({
            "command":"$AP0:130X127Y!",
            "executionTime":"8000"
          }));
          highlightleft.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightleft.style.backgroundColor = 'white';
          }, 1 * 1000);
        }

        else if(event.keyCode === 39){
          let highlightright: HTMLElement = this.el.nativeElement.querySelector("#rightbtn");
          this.tankcontrol.sendControl(JSON.stringify({
            "command":"$AP0:67X127Y!",
            "executionTime":"8000"
          }));
          highlightright.style.backgroundColor = 'yellow';
          window.setTimeout(function() {
            highlightright.style.backgroundColor = 'white';
          }, 1 * 1000);
        }
  }

}
