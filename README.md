<h1> How to Programming </h1>

**Made for FRC Team 972: Iron Claw, in Los Gatos High School, CA.**  
**Authored by** [**@whackamadoodle3000**](https://github.com/whackamadoodle3000)__,__ [**@me1234q**](https://github.com/me1234q)  ,  [**TsarF**](https://github.com/TsarF)

## Contents
[Guides](#guides)   
&emsp; [Motors](#use-motors)
&emsp; [Voltage Compensation](#voltage-compensation)
&emsp; [Joysticks](#use-joysticks)  
&emsp; [Encoders](#use-external-encoders)  
&emsp; [Built-in Accelerometer](#use-the-built-in-accelerometer)  
&emsp; [NavX](#use-the-kauai-labs-navx)  
&emsp; [Solenoids](#use-single-acting-solenoids)  
&emsp; [Double Solenoids](#use-a-double-solenoid)  
&emsp; [Compressors](#use-a-compressor)  
&emsp; [Limit Switches](#use-a-limit-switch)
&emsp; [Start Programming On Your Computer](#start-programming-on-your-device)

[PID](#PID)
&emsp; [Use The Inbuilt PID in the Falcon Motors](#inbuilt-falcon-pid)    


[Examples](#examples)  
&emsp; [Make Motor Spin Based on Joystick Input](#make-motor-spin-based-on-joystick-input)  
&emsp; [Arcade Drive](#arcade-drive)  
&emsp; [Tank Drive](#tank-drive)  
&emsp; [PID Example Snippet](#pid-example-snippet)  
&emsp; [Toggling Double Solenoids](#toggling-double-solenoids)  

## Guides

### Use Motors
**Motors are one of the most common actuators and provide rotational motion. When given power, the motor will spin with speed proportional to the voltage and torque proportional to the current. They are controlled with a talon.**

#### TalonFX motor controllers (falcon500)
- Note: we often use the WPI version of this class which provides more compatibility with WPI libraries. So, use the class WPI_TalonFX instead of TalonFX.
- To import
	- To import TalonFX `import com.ctre.phoenix.motorcontrol.can.*;`
	- To import control modes `import com.ctre.phoenix.motorcontrol.ControlMode;`
- To initialize
	- To initialize motor `TalonFX Motor = new TalonFX(6);`
- To use
	- Set motor absolute power `Motor.set(ControlMode.PercentOutput, value);` with value as double between -1 and 1
	- Set motor to follow another `Motor.set(ControlMode.Follower, value);` with value as the id of the other talon
#### TalonSRX motor controllers
- To import
	- To import TalonSRX `import com.ctre.phoenix.motorcontrol.can.*;`
	- To import control modes `import com.ctre.phoenix.motorcontrol.ControlMode;`
- To initialize
	- To initialize motor `TalonSRX Motor = new TalonSRX(6);`
	- To initialize automatic deadband `Motor.enableDeadbandElimination(true);`
- To use (with various control modes)
	- Set motor absolute power `Motor.set(ControlMode.PercentOutput, value);` with value as double between -1 and 1
	- Set motor to follow another `Motor.set(ControlMode.Follower, value);` with value as the id of the other talon
- Documentation: http://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___talon_s_r_x.html

#### SparkMAX motor controllers
- To import
	- To import SparkMAX `import com.revrobotics.CANSparkMax;`
	- To import motor Types `import com.revrobotics.CANSparkMaxLowLevel.MotorType;`
- To initialize
	- To intialize SparkMAX controller `CANSparkMax <name> = new CANSparkMax(<MotorID (setup with REV Robotics SparkMAX client)>, <MotorType.kBrushless OR MotorType.kBrushed>);`
		- There are multiple control modes for different motors because some motors are ***BRUSHED*** and some are ***BRUSHLESS*** you can tell if it is brushed or brushless by counting the wires going into the motor from the controller
			- 3 wires: Brushless
			- 2 wires: Brushed
			- ***NOTE*** Some motors, like NEO motors, have 3 wires ***AND*** what may look like a 4th wire, which is actually the encoder wire, inside the protective sleeve, there are 6 more wires. DO NOT BE CONFUSED BY THAT.
- To use
	- To RUN the motor `CANSparkMax.set(<speed 0.0 - 1.0>);`
	- Set Follower `CANSparkMax.follow(<motor to follow>)` or `CANSparkMax.follow(<motor to follow>, <inverted? true:false>);`
	- Encoder `CANSparkMax.getEncoder();`
		- Get velocity `CANSparkMax.getEncoder().getVelocity();`
		- Get position `CANSparkMax.getEncoder().getPosotion();`
		- SET position `CANSparkMax.getEncoder().setPosition(<position (double)>); //used for aeroid out the encoders, and resetting field position`

### Voltage Compensation
**Voltage Compensation allows your motor to run with the same amount of power with less battery voltage**
- To use, make SURE your set the max voltage of your battery before your enable voltage compensation. This lets the TalonFX know the maximum voltage to apply to the motor `m_motor.configVoltageCompSaturation(Constants.kRobotVoltage);`
- To enable voltage compensation `m_motor.enableVoltageCompensation(true);`


## PID

### Inbuilt Falcon Pid
**WPIlib contains the pid motor controller class and we can calculate the PID values in the rio and send them via CAN to the falcon motor. However, by doing all of this within the TalonFX motor controller, the motor controller that controls the falcon we a) do not have to send values via CAN so it is faster, and b) we can initialize multiple PIDs on one motor with their own unique kP, kI, and kD values for different tasks and can select the one desired.**

#### the config_kP, config_kI, config_kD, and config_kF commands: 
- configure the kP value for a PID: `m_motor.config_kP(PIDslot, caseOnekP);`
- Configure the kI value for a PID: `m_motor.config_kI(PIDslot, caseOnekI);`
- Configure the kD value for a PID: `m_motor.config_kD(PIDslot, caseOnekD);`
- Configure the feedforward(kF) value for a PID:`m_motor.config_kF(PIDslot, caseOnekF);`
- PIDslot is the slot(a number) that represents your PID. You refrence the number when you want to add a setpoint to that PID, for example. 
- caseOnePIDkP, caseOnPIDkI, caseOnePIDkD, and caseOnekF are all constants for a specific PID

### Use Joysticks
**Joysticks are (misleadingly) an umbrella term for all user input devices, including gamepads, joysticks, etc. Joystick objects can receive joystick and button input.**
- To import `import edu.wpi.first.wpilibj.Joystick;`
- To initialize `Joystick Joy = new Joystick(0);`
- To use
	- To get axis values `Joy.getRawAxis(0);` with value between -1 and 1
	- To get button values `Joy.getRawButton(0);` with boolean output
	- To get angle in radians `Math.atan2(xAxis, -yAxis)`
		- To get angle in degrees `Math.atan2(xAxis, -yAxis) * 180 / Math.PI`
	- To get magnitude `Math.sqrt(xAxis*xAxis + yAxis*yAxis);`
- Documentation: http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Joystick.html

### Use External Encoders
**Encoders are used for measuring rotation. This is done through counting the number of "clicks", which increments or decrements with every slight rotation.**
- To import `import edu.wpi.first.wpilibj.Encoder`
- To initialize `Encoder sampleEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);`
	- To set max period (secs) without clicks before it is considered at rest `sampleEncoder.setMaxPeriod(.1);`
	- Minimum rate (rotation/time) before device is stopped `sampleEncoder.setMinRate(10);`
	- How much distance is traveled every encoder pulse `sampleEncoder.setDistancePerPulse(5);`
	- Reverse the direction encoder counts `sampleEncoder.setReverseDirection(true);`
	- How many samples to take (1-127) when determining the period`sampleEncoder.setSamplesToAverage(7);`
- To use
	- To get count of clicks `int count = sampleEncoder.get();`
	- To get distance `double distance = sampleEncoder.getDistance();`
	- To get period between clicks `double period = sampleEncoder.getPeriod();`
	- To get speed `double rate = sampleEncoder.getRate();`
	- To get current direction `boolean direction = sampleEncoder.getDirection();`
	- To see if the encoder is stopped turning `boolean stopped = sampleEncoder.getStopped();`
- Documentation: http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Encoder.html
- More info: https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599717-encoders-measuring-rotation-of-a-wheel-or-other-shaft
### Use Talon Encoders
**These encoders function the same as [external encoders](#use-encoders), but are connected to the talon's encoder port and thus can be directly accessed from the talon object.**
- To import `import com.ctre.phoenix.motorcontrol.FeedbackDevice;`
- To initialize `talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);`
- To use 
	- `talon.getSensorCollection().getQuadraturePosition();`
	- `talon.getSensorCollection().getQuadratureVelocity();`
- Documentation: http://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_sensor_collection.html

### Use Absolute Encoders
**These encoders tell you the encoder position but they do NOT get reset every time we power on the robot. The encoder remembers it's position. These encoders give you values in ROTATIONS. One 360 degree rotation of the encoder is one rotation. As we rotate an absolute encoder past 360 degrees, the value gets reset to 1 rotation and goes down from there. Therefore it is important to position an absolute encoder such that the reset does not happen.**
- Import the library `import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;`
- To create the encoder  `DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(roborio_DIO_port_the_encoder_is_plugged_into);`
- To get the encoder distance(will be in ROTATIONS) `m_absoluteEncoder.getDistance()`
- There are other useful functions available to you. 
- Documentation(scroll down): https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html


### Use the Built-in Accelerometer
**This is the RoboRio's built-in accelerometer. It allows you to get the acceleration in the x, y, and z directions.**
- To import `import edu.wpi.first.wpilibj.BuiltInAccelerometer;`
- To initialize `BuiltInAccelerometer accel = new BuiltInAccelerometer();`
- To use
	- Get x acceleration `accel.getX()`
	- Get y acceleration `accel.getY()`
	- Get z acceleration`accel.getZ()`
- Documentation: http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/BuiltInAccelerometer.html

### Use the Kauai Labs NavX
**The NavX gives a variety of information on the robot state, including orientation, velocity, acceleration, position, and more.**
- To import 
	- `import com.kauailabs.navx.frc.AHRS;`
	- `import edu.wpi.first.wpilibj.SPI;` or I2C
- To initialize
	- If mounted on RoboRIO `AHRS ahrs = new AHRS(SPI.Port.kMXP);`
	- If mounted elsewhere `AHRS ahrs = new AHRS(I2C.Port.kMXP);`
- To use
	- Orientation Data
		- To get heading `ahrs.getAngle()`
		- To get yaw `ahrs.getYaw()` (-180 to 180 degrees)
		- To get pitch `ahrs.getPitch()` (-180 to 180 degrees)
		- To get roll `ahrs.getRoll()`
		- To get compass data `ahrs.getCompassHeading()` (0 to 360)
	- Velocity Data in Meters/Sec
		- Velocity in x direction `ahrs.getVelocityX()`
		- Velocity in y direction `ahrs.getVelocityY()`
		- Velocity in z direction `ahrs.getVelocityZ()`
		- Rate of yaw (turning) `ahrs.getRate()`
	- Acceleration Data in g-force
		- Acceleration in x direction `ahrs.getWorldLinearAccelX()`
		- Acceleration in y direction `ahrs.getWorldLinearAccelY()`
		- Acceleration in z direction `ahrs.getWorldLinearAccelZ()`
	- Distance/Displacement Data in meters
		- Displacement x `ahrs.getDisplacementX()`
		- Displacement y `ahrs.getDisplacementY()`
		- Displacement z `ahrs.getDisplacementZ()`
	- Pressure and Temperature data
		- Barometric Pressure in millibars `ahrs.getBarometricPressure()`
		- Temperature in Celsius `ahrs.getTempC()`
	- Boolean Motion Data
		- To see if it is rotating `ahrs.isRotating()`
		- To see if it is moving `ahrs.isMoving()`
	- Reset
		- To reset measurements for the yaw gyro `ahrs.reset()`
		- To zero yaw gyro `ahrs.zeroYaw()` (tell it what forward mean)
		- To reset displatement distances `ahrs.resetDisplacement()`
	- Board Info
		- To get update rate `ahrs.getActualUpdateRate()`
		- Check if it is connected `ahrs.isConnected()`
- Documentation: https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
	
### Use Single Acting Solenoids
**Solenoids are the most often used actuator for linear motion. Single acting Solenoids extend by increasing the air pressure inside a piston. [Double Solenoids](#use-a-double-solenoid) are recommended, as their have two channels, allowing them to more easily switch between extended and retracted position.**
- To import `import edu.wpi.first.wpilibj.Solenoid;`
- To initialize `Solenoid solenoid = new Solenoid(1)`
- To use 
	- To turn on `solenoid.set(true)`
	- To turn off `solenoid.set(false)`
	- To get solenoid state in boolean `solenoid.get()`  
- Documentation: http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Solenoid.html
	
### Use a Double Solenoid 
**These are similar to single acting solenoids, but have a second channel allowing them to revert to their original position without the need of an external force.**
- To import `import edu.wpi.first.wpilibj.DoubleSolenoid;`
- To initialize `DoubleSolenoid doubleSolenoid = new DoubleSolenoid(forwardchannel, reversechannel);`
- To use
	- To block all pressure `doubleSolenoid.set(DoubleSolenoid.Value.kOff);`
	- To put pressure in forward channel `doubleSolenoid.set(DoubleSolenoid.Value.kForward);`
	- To put pressure in reverse channel `doubleSolenoid.set(DoubleSolenoid.Value.kReverse);`  
- Documentation: http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/DoubleSolenoid.html
### Use a Compressor
**Compressors are used for compressing air. They are need for any pneumatics, as they need air at a high pressure to function.**
- To import `import edu.wpi.first.wpilibj.Compressor;`
- To initialize `Compressor c = new Compressor(40);`
- To control
	- To toggle closed loop control (goes up until maximum PSI) `c.setClosedLoopControl(boolean);`
	- To turn on `c.start();`
	- To turn off `c.stop();`
- Getting information
	- To get if the pressure is low in boolean `c.getPressureSwitchValue()`
	- To get the current being consumed in amps in double `c.getCompressorCurrent()`
	- To check if closed loop control is on `c.getClosedLoopControl()`
- Getting fault/error information
	- Check if compressor is disabled because current is too high `c.getCompressorCurrentTooHighFault()`
	- Check if the compressor is disabled because output is shorted `c.getCompressorShortedFault()`
	- Check if compressor is is not connected/not drawing enough current `c.getCompressorNotConnectedFault()`  
- Documentation: http://first.wpi.edu/FRC/roborio/beta/docs/java/edu/wpi/first/wpilibj/Compressor.html

### Use a Limit Switch
**Limit switches are devices that mechanically prevent an actuator from extending a certain predetermined position.**
- To import `import edu.wpi.first.wpilibj.DigitalInput;`
- To initialize `DigitalInput limitSwitch = new DigitalInput(1);`
- To get value `limitSwitch.get()` (boolean)  
- Documentation: http://first.wpi.edu/FRC/roborio/beta/docs/java/edu/wpi/first/wpilibj/DigitalInput.html

### Start Programming On Your Device
#### Install Git
Here you can Install Git Desktop or Git Bash, whichever one you prefer.
- Download link: [link](https://central.github.com/deployments/desktop/desktop/latest/win32)
- Run the installer
- Make sure ``Add to PATH`` option is selected

#### Download and Install Java 11 SDK
- Download the installer from this [link](https://www.oracle.com/technetwork/java/javase/downloads/jdk11-downloads-5066655.html)
- Run the installer, make sure ``Add to PATH`` option is selected
- Thats it

#### Setting Up Computer Vision Development Environment
Python is usually used for computer vision stuff, it is very useful for automatic allignment
- Download Python 3 from here: [link](https://www.python.org/ftp/python/3.7.4/python-3.7.4-amd64.exe)
- Run the Installer
- Ensure the ``Add to PATH`` option is selected, otherwise it will not work
- Ensure everything is installed correctly by running ``python`` or ``python3`` in the command line
- If an interactive python inveronment opens, you are good to go

Installing OpenCV package for python
- Open the command line once again and run ``python -m pip install`` if that fails, then run ``py -m pip install``
- Wait for it to install, accept any warning/messages

#### Installing FRC VS Code
This IDE is different from the others as it has integrated WPI lib and vendor library manage for easier installation of new libraries for new devices.
- You want to download the installer: [link](https://github.com/wpilibsuite/allwpilib/releases/download/v2019.4.1/WPILibInstaller_Windows64-2019.4.1.zip)
- Extract the files to any folder of you choise and run the installer file (.msi or .exe)
- In the installer, make sure it is installing a *NEW* version of VS CODE
- wait for the installation to finish

#### Setting up the IDE
- Configure your settings as you wish
- Configure the team number to 972
#### Installing CTRE Toolsuite (Optional)
This toolsuite is often used for configuring Driver Station dashboards, testing camera feed and configuring CAN devices
- Get the installer zip file from here: [link](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/download/Phoenix_v5.15.0.0/CTRE.Phoenix.Framework.v5.15.0.1.zip)
- Run the installer, it will install the following: Phoenix Tuner, Smart Dashboard

#### Installing the NI Toolsuite
This one comes with the Driver Station and RoboRIO Imaging Tool
- Go to the following [link](http://www.ni.com/download/first-robotics-software-2017/7904/en/)
- Click on the download link to download the latest update suite
- Run the installers, login with the email: ``garyk@dslextreme.com`` and password: ``garyk``

#### REV Robotics SparkMAX Client
This application is used for configuring SparkMAX motor controllers, updating their firmware and running motors for testing
- Go to the download [link](http://www.revrobotics.com/content/sw/max/client/spark-max-client-setup-1.0.0.exe)
- Run the installer

#### REV Robotics Libraries
Used for controlling the SparkMAX motors
- In FRC VS CODE press ``ctrl + shift + p`` to open the terminal
- Type in ``Manage Vendor Libraries``
- Give it this link: [https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json](https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json)
- Press enter
- Recompile to ensure library was installed correctly

### NOTE: SOME OF THE INSTALLERS ARE DOWNLOADED AS ``.zip`` FILES WHICH YOU NEED TO EXTRACT TO INSTALL PROPERLY


## Examples

#### Make Motor Spin based on Joystick Input　

```java
package org.usfirst.frc.team972.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {
	WPI_TalonSRX motor = new WPI_TalonSRX(6); //initialize motor
	Joystick joy = new Joystick(0); //initialize joystick (to find number, check driver station)
	double powerMultiplier = 0.7; //scales down motor power

	public void robotInit() {
		//This function is run when the robot is first started up
	}

	public void teleopPeriodic() {
		//This function is called periodically during teleoperated control
		double joystickValue = joy.getRawAxis(1); //find axis number in driver station
		System.out.println("Joy: " + joystickValue);
		Motor.set(joystickValue*powerMultiplier); //scale down by powerMultiplier
	}
}
```


#### Arcade Drive　

```java
package org.usfirst.frc.team972.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot {

	/* talons for arcade drive */
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(6);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(2);

	/* extra talons and victors for six motor drives */
	WPI_TalonSRX _leftSlave1 = new WPI_TalonSRX(5);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(7);
	WPI_TalonSRX _leftSlave2 = new WPI_TalonSRX(4);
	WPI_VictorSPX _rightSlave2 = new WPI_VictorSPX(17);

	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);

	Joystick _joy = new Joystick(0);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		/*
		 * take our extra talons and just have them follow the Talons updated in
		 * arcadeDrive
		 */
		_leftSlave1.follow(_frontLeftMotor);
		_leftSlave2.follow(_frontLeftMotor);
		_rightSlave1.follow(_frontRightMotor);
		_rightSlave2.follow(_frontRightMotor);

		/* drive robot forward and make sure all 
		 * motors spin the correct way.
		 * Toggle booleans accordingly.... */
		_frontLeftMotor.setInverted(false);
		_leftSlave1.setInverted(false);
		_leftSlave2.setInverted(false);
		
		_frontRightMotor.setInverted(false);
		_rightSlave1.setInverted(false);
		_rightSlave2.setInverted(false);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		double forward = -1.0 * _joy.getY();
		/* sign this so right is positive. */
		double turn = +1.0 * _joy.getZ();
		/* deadband */
		if (Math.abs(forward) < 0.10) {
			/* within 10% joystick, make it zero */
			forward = 0;
		}
		if (Math.abs(turn) < 0.10) {
			/* within 10% joystick, make it zero */
			turn = 0;
		}
		/* print the joystick values to sign them, comment
		 * out this line after checking the joystick directions. */
		System.out.println("JoyY:" + forward + "  turn:" + turn );
		/* drive the robot, when driving forward one side will be red.  
		 * This is because DifferentialDrive assumes 
		 * one side must be negative */
		_drive.tankDrive(forward, turn);
	}
}
```

#### Tank Drive　

```java
package org.usfirst.frc.team972.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot {

	/* talons for arcade drive */
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(6);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(2);

	/* extra talons and victors for six motor drives */
	WPI_TalonSRX _leftSlave1 = new WPI_TalonSRX(5);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(7);
	WPI_TalonSRX _leftSlave2 = new WPI_TalonSRX(4);
	WPI_VictorSPX _rightSlave2 = new WPI_VictorSPX(17);

	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);

	Joystick leftJoy = new Joystick(0);
	Joystick rightJoy = new Joystick(1);

	//This function is run when the robot is first started up and should be used for any initialization code.
	
	public void robotInit() {
		//take our extra talons and just have them follow the Talons updated in
		_leftSlave1.follow(_frontLeftMotor);
		_leftSlave2.follow(_frontLeftMotor);
		_rightSlave1.follow(_frontRightMotor);
		_rightSlave2.follow(_frontRightMotor);

		/* drive robot forward and make sure all 
		 * motors spin the correct way.
		 * Toggle booleans accordingly.... */
		_frontLeftMotor.setInverted(false);
		_leftSlave1.setInverted(false);
		_leftSlave2.setInverted(false);
		
		_frontRightMotor.setInverted(false);
		_rightSlave1.setInverted(false);
		_rightSlave2.setInverted(false);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
	
		double right = rightJoy.getY();
		double left =  leftjoy.getY();
		/* deadband */
		if (Math.abs(left) < 0.10) {
			/* within 10% joystick, make it zero */
			left = 0;
		}
		if (Math.abs(right) < 0.10) {
			/* within 10% joystick, make it zero */
			right = 0;
		}
		/* print the joystick values to sign them, comment
		 * out this line after checking the joystick directions. */
		System.out.println("Left: " + left + "  Right:" + right );
		_drive.tankDrive(left, right);
	}
}
```
#### PID Example Snippet

```java
//At top:


double proportionFactor = 0.2; //These values should be tuned to be the most efficient.
double integralFactor = 0.2;
double derivativeFactor = 0.2;
double marginOfError = 0.01;

double actual = 1; //Change based on situation
double desired = 10;

double priorActual = actual;
double integral = 0;
double error = desired - actual;

Repeated loop (like slow or fast periodic) {
	priorActual = actual;
	actual = <get sensor data>;
	error = desired - actual;
	if (Math.abs(desired-actual) < marginOfError){
		<End of PID, Position Achieved>
	} else {
		integral = integral + current - desired;
	}
	return proportionFactor*error + integralFactor*integral + derivativeFactor*(actual-priorActual);
}

```
#### Toggling Double Solenoids  

```java
package org.usfirst.frc.team972.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.*;

public class Robot extends IterativeRobot {

	Compressor com = new Compressor(40);
	DoubleSolenoid sol = new Double Solenoid(1, 2);

	Joystick joy = new Joystick(1);

	//This function is run when the robot is first started up and should be used for any initialization code.
	
	public void robotInit() {
		//set up compressors
		com.setClosedLoopControl(true);
		com.start();
	}

	
	public void teleopPeriodic() {
		// Called when the button was released since last check
		if(joystick.getRawButtonReleased(0)) {
			if(frontSolenoid.get().equals(kForward)) {
				frontSolenoid.set(kReverse);
			} else if(frontSolenoid.get().equals(kReverse)) {
				frontSolenoid.set(kForward);
			}
			
		}
	}
}
```
<hr />
Last Edit on January 26, 2019
