import { Component, OnInit } from '@angular/core';
import { DomSanitizer } from "@angular/platform-browser";
import { Location } from '@angular/common';

@Component({
  selector: 'app-camera',
  templateUrl: './camera.component.html',
  styleUrls: ['./camera.component.less']
})
export class CameraComponent implements OnInit {
  camera: { url: any };

  constructor(private sanitizer: DomSanitizer) {
    this.camera = { url: "http://" + location.hostname + "/camera" };
  }

  secureUrl() {
    console.log("cameraUrl:" + this.camera.url);
    return this.sanitizer.bypassSecurityTrustResourceUrl(this.camera.url);
  }

  ngOnInit() {
  }

}
