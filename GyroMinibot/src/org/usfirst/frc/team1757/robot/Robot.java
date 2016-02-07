
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
   double Kp = 0;
   double Ki = 0;
   double Kd = 0;
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
		pidLeft = new PIDController(1, Kp, Ki, Kd, gyro, leftMotor);
		//  adjust value of 1, multiplies source by 1
		pidRight = new PIDController(-1, Kp, Ki, Kd, gyro, rightMotor);
		// adjust value of -1, multiplies source by -1
		rightMotor.setInverted(true);
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
    		
    		//Gyro Code Bruh
    		angle = gyro.getAngle(); //get heading
    		//myRobot.drive(gamepad.getY()*0.4, -angle*Kp);
    		
    		pidLeft.setSetpoint(gamepad.getY() *45);
    		pidRight.setSetpoint(gamepad.getY()* -45);    		
    		//  this is the inverted motor, so set negative setpoint
    		
    		//pidLeft.setPID(.003, 0.0, 0.0, gamepad.getY());
    		//pidRight.setPID(.003, 0.0, 0.0, gamepad.getY());
    		
    		//pidLeft.setSetpoint(0);
    		//pidRight.setSetpoint(0);
    		
    		//Good starting Kp is 0.03!!!!!!!!
    			// OK so the correction system works, now we need to somehow add and subtract from driving values
    		
    		//pidLeft.setPID(Kp, 0, 0);
    		//pidRight.setPID(Kp, 0, 0);
    		
    		
    		
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