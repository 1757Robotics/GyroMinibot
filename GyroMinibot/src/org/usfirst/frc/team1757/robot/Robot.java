
package org.usfirst.frc.team1757.robot;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
//import edu.wpi.first.wpilibj.PIDController;
import org.usfirst.frc.team1757.robot.PIDController;

public class Robot extends SampleRobot {
   CANTalon leftMotor, rightMotor;
   ADXL362 accel;
   ADXRS450_Gyro gyro;
   Joystick gamepad;
   RobotDrive myRobot;
   double Kp = 0.04;
   double Ki = 0;
   double Kd = 0.08;
   double Kf = 0;
   double angle = 0;
   PIDController pidLeft;
   PIDController pidRight;
   
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
    		SmartDashboard.putNumber("Kp", Kp);
    		SmartDashboard.putNumber("Ki", Ki);
    		SmartDashboard.putNumber("Kd", Kd);
    		SmartDashboard.putNumber("Kf", Kf);
    		SmartDashboard.putNumber("Angle", gyro.getAngle());
    		
    		angle = gyro.getAngle(); //get heading
    		//myRobot.drive(gamepad.getY()*0.4, -angle*Kp);
    		
    		//pidLeft.setSetpoint(gamepad.getY() *45);
    		//pidRight.setSetpoint(gamepad.getY()* -45);    		
    		//  this is the inverted motor, so set negative setpoint
    		
    		//pidLeft.setPID(.020, 0.0, 0.080, gamepad.getY());
    		//pidRight.setPID(.020, 0.0, 0.080, gamepad.getY());
    		//  feed forward did not calculate...
    		
    		pidLeft.setSetpoint(0);
    		pidRight.setSetpoint(0);
    		
    		pidLeft.setDrive(gamepad.getY());
    		pidRight.setInverted(true);
    		pidRight.setDrive(gamepad.getY());
    		
    		
    		
    		if (gamepad.getRawButton(7) == true){
   			  Kp -= .0001;
   		  	}
   		  	if (gamepad.getRawButton(8) == true){
   			  Kp += .0001;
   		  	}
   		  	if (gamepad.getPOV(0)== 0){
			  Kp += .1;
  		  	}
   		  	if (gamepad.getPOV(0)== 90){
			  Kp += .01;
   		  	}
   		  	if (gamepad.getPOV(0) == 180){
			  Kp -= .1;
   		  	}
   		  	if (gamepad.getPOV(0) == 270){
   		  		Kp -= .01;
   		  	}
   		  	if (gamepad.getRawButton(1) == true){
   		  		gyro.reset();
   		  	}
   		  	if (gamepad.getRawButton(3) == true){
   		  		Kp = 0;
   		  	}
    		
    	}
    	
    }
    

}