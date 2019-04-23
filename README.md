# closed_loop_motor_control
PID control of a unipolar motor using an accelerometer with MSP430FG4618

Final Project: Closed Loop Control of a Stepper Motor

Project Goal and Theory:
	To control a unipolar stepper motor’s position using a PID control loop (Proportional Integral Derivative). This is accomplished by using an accelerometer. The accelerometer provides angular position data that the motor can then use to move to a desired position.
	Angular position in a 2D plane (X-Z plane) can be determined using an accelerometer. This is done by taking the inverse tangent of the acceleration in the Z and the X directions, with the assumption that the magnitude of the acceleration is equal to 1 g. Any outside disturbance that increases the magnitude of the acceleration will introduce error. We also assume that the accelerometer’s X and Y vectors are perfectly aligned in the X-Z plane of 3D space. Any tilt in the Y direction introduces error, which increases with the amount of tilt there is.
	In the PID control loop, we calculate the difference between the current position of the motor, given by the accelerometer, and the desired position that is set in software. This difference is called the error and is given by the equation:
error=Set Point-Process Variable(t)
Where the set point is the desired position and the process variable is the current position. 
With this error, we can derive the Proportional, Integral, and Derivative terms. The proportional term is equal to the current error, the integral term accounts for past error, and the derivative term accounts for the current rate of change of the error. The combination of these inputs drive the PWM signal frequency that drives the motor. This combination, which is called the manipulated variable, can be represented in general terms by the equation:
MV(t)=Kp*Proportional+Ki*Integral+Kd*Derivative
Where Kp, Ki, and Kd are gain values for each term that can be manually set to tune the controller. By setting individual gains Ki and Kd to zero, we can simplify the control loop to proportional only. However, proportional only control can have oscillations about the set point or steady state error, depending on the gain set. Or we can set only Kd to zero, giving a proportional integral control loop. Ideally, with all three terms, we can tune the control loop in to have fast response, small steady state error, and little to no oscillation about the set point. These conditions are tuned by changing each individual gain to optimize the output.
Required Components:
	MSP430 Experimenter’s board
	Accelerometer ADXL335
	28BJY-48 Stepper Motor and ULN2003 Driver
	Separate 5v power source for motor
	Jumper cables
	3D printed mounting bracket and stand
	Breadboard

Connecting Components:

	Connect the GND and Vcc wires of the 5v power source to the negative and positive rails of the breadboard. Use a jumper cable to make the negative rail common ground with the MSP430.
	Connect jumper cables to the accelerometer’s GND, Vcc, X and Z pins. Then connect the GND of the accelerometer to common ground on the breadboard and Vcc of the accelerometer to the Vcc of the MSP430. Connect X to P6.4 and Z to P6.6 of the MSP430. The ports are located on Header 8 as pin 5(P6.4) and pin 7(P6.6).

	Connect the stepper motor to the ULN2003 driver. Connect the Vcc of the ULN2003 to the 5v rail of the breadboard, and GND pin to ground on the breadboard. There are 4 input signal pins labeled IN1, IN2, IN3 and IN4. Connect IN1 to P2.0, IN2 to P2.3, IN3 to P2.6, and IN4 to P2.7. It is important to connect them in the proper sequence for the motor to work. These ports are connected to pins 1(P2.0), 4(P2.3), 7(P2.6) 8(P2.7) of Header 4.
