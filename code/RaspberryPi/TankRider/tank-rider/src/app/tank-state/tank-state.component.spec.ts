import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { TankStateComponent } from './tank-state.component';

describe('TankStateComponent', () => {
  let component: TankStateComponent;
  let fixture: ComponentFixture<TankStateComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ TankStateComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(TankStateComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
