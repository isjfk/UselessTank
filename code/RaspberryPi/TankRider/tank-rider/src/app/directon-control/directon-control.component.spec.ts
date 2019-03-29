import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { DirectonControlComponent } from './directon-control.component';

describe('DirectonControlComponent', () => {
  let component: DirectonControlComponent;
  let fixture: ComponentFixture<DirectonControlComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ DirectonControlComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(DirectonControlComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
