# Servo Test for Pan-Tilt HAT
1. Build in RaspberryPi:
    ```cmd
	make
	```
1. Command arguments:
    ```cmd
	./main <range> <inc> <interval>
	```
	* range: Servo angle move range in degree.
	    * Servo angle will move from 90-range to 90+range.
		* Default value: 50.
	* inc: Servo angle incremental in degree in each interval.
	    * Default value: 1
	* interval: Interval time of each servo value change in ms.
	    * Default value: 50
1. Usage Example:
    ```cmd
	./main 30 5 1000
	```
	Move the servo angle from 60 degree to 120 degree, increase the angle 5 degree each 1000 ms.
1. Before assemble the servos, execute following command:
    ```cmd
	./main 0
	```
	This will move all servos to center position.
