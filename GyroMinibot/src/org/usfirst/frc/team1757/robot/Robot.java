
package org.usfirst.frc.team1757.robot;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.CameraServer;
import org.usfirst.frc.team1757.robot.PIDController;

public class Robot extends SampleRobot {
   CANTalon leftMotor, rightMotor;
   ADXL362 accel;
   ADXRS450_Gyro gyro;
   Joystick gamepad;
   RobotDrive myRobot;
   CameraServer camera;
   double Kp = 0.04;
   double Ki = 0;
   double Kd = 0.08;
   double Kf = 0;
   PIDController pidLeft;
   PIDController pidRight;
   double setpoint = 0;
   double initialTurn = 10;
   double turnConstant = 0.4;
   
	public Robot(){
		gamepad = new Joystick(0);
		leftMotor = new CANTalon(1);
		rightMotor = new CANTalon(2);
		gyro = new ADXRS450_Gyro();
		accel = new ADXL362(Accelerometer.Range.k8G);
		
		//myRobot = new RobotDrive(leftMotor, rightMotor);
		//gyro.setPIDSourceType(PIDSourceType.kRate);
		pidLeft = new PIDController(0, Kp, Ki, Kd, Kf, gyro, leftMotor);
		pidRight = new PIDController(0, Kp, Ki, Kd, Kf, gyro, rightMotor);
		//rightMotor.setInverted(true);
		
		SmartDashboard.putNumber("Constant", turnConstant);
		gyro.reset();
		
		camera = CameraServer.getInstance();
        camera.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        camera.startAutomaticCapture("cam0");
	}
   
    public void operatorControl() {
    	while (isOperatorControl() && isEnabled()){
    		 
    		// Smartdashboard
    		SmartDashboard.putData("Gyro", gyro);
    		SmartDashboard.putData("Accelerometer", accel);
    		SmartDashboard.putData("pidLeft", pidLeft);
    		SmartDashboard.putData("pidRight", pidRight);
      		SmartDashboard.putNumber("rightMotor", rightMotor.get());
    		SmartDashboard.putNumber("leftMotor", leftMotor.get());
    		SmartDashboard.putNumber("Right Joystick", gamepad.getRawAxis(3)*0.4 );
    		SmartDashboard.putNumber("Left Joystick", gamepad.getY()*0.4);
    		SmartDashboard.putNumber("Angle", gyro.getAngle());
    		SmartDashboard.putNumber("Constant", turnConstant);
    		turnConstant = SmartDashboard.getNumber("Constant");   		
    		
    		
    		pidLeft.setDrive(gamepad.getY());
    		pidRight.setInverted(true);
    		pidRight.setDrive(gamepad.getY());
    		
    		//method #1 - use if robot goes rogue
    		/*
    		if (gamepad.getRawAxis(3) > .1) {
    			pidRight.setSetpoint(initialTurn);
    			pidLeft.setSetpoint(initialTurn);
    			Timer.delay(.01);
    			gyro.reset();
    		}
    		if (gamepad.getRawAxis(3) < -.1){
    			pidRight.setSetpoint(-initialTurn);
    			pidLeft.setSetpoint(-initialTurn);
    			Timer.delay(.01);
    			gyro.reset();
    		}
    		*/
    		
    		//method #2
    		if (gamepad.getRawAxis(3) > .1) {
    			setpoint += gamepad.getRawAxis(3)*turnConstant;
    			pidLeft.setSetpoint(setpoint);
    			pidRight.setSetpoint(setpoint);
    		}
    		if (gamepad.getRawAxis(3) < -.1) {
    			setpoint += gamepad.getRawAxis(3)*turnConstant;
    			pidLeft.setSetpoint(setpoint);
    			pidRight.setSetpoint(setpoint);
    		}
   		  	if (gamepad.getRawButton(1) == true){
   		  		gyro.reset();
   		  	}
    		
    	}
    	
    }
    

}