package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.command.Command;

public class FieldCentricSwerveDrive extends Command {
	
	public static final double OMEGA_SCALE = 1.0 / 30.0;
	public static final double DEADZONE = 0.05;

	private double originHeading = 0.0;
	private double originCorr = 0;
    private double leftPow = 1.0;
	private double rightPow = 1.0;
	private double correctedHeading = 0;
	private boolean targetAngleAquired = false;
	private double orientationAngle = 0;
	private boolean leftOuterTrack = false;
	private boolean leftMiddleTrack = false;
	private boolean leftCenterTrack = false;
	private boolean rightCenterTrack = false;
	private boolean rightMiddleTrack = false;
	private boolean rightOuterTrack = false;

	public FieldCentricSwerveDrive() {
        requires(Robot.drive);
        //originHeading = RobotMap.navX.getFusedHeading();
    }
	
	@Override
	protected void initialize() {
		originHeading = Robot.zeroHeading;
		//originCorr = Robot.zeroAngle;
	}

    @Override
	protected void execute() {
		if (Robot.oi.getLeftJoyButton(7)) {
			originHeading = RobotMap.navX.getFusedHeading();
		}

		double originOffset = 360 - originHeading;
		originCorr = RobotMap.navX.getFusedHeading() + originOffset;
		correctedHeading = originCorr % 360;

		//Track boolean values are inverted as input from sensors is false when line 
		//is present, and true when no line is present.
		leftOuterTrack = !RobotMap.leftOuterLineTracker.get();
		leftMiddleTrack = !RobotMap.leftMiddleLineTracker.get();
		leftCenterTrack = !RobotMap.leftCenterLineTracker.get();
		rightCenterTrack = !RobotMap.rightCenterLineTracker.get();
		rightMiddleTrack = !RobotMap.rightMiddleLineTracker.get();
		rightOuterTrack = !RobotMap.rightOuterLineTracker.get();
		SmartDashboard.putNumber("OriginHeading", originHeading);
		//SmartDashboard.putNumber("OriginCorrection", originCorr);
		//SmartDashboard.putNumber("OriginOffset", originOffset);
		SmartDashboard.putNumber("CorrectedHeading", correctedHeading);	
		SmartDashboard.putBoolean("Left Outer Tracker", leftOuterTrack);
		SmartDashboard.putBoolean("Left Middle Tracker", leftMiddleTrack);
		SmartDashboard.putBoolean("Left Center Tracker", leftCenterTrack);	
		SmartDashboard.putBoolean("Right Center Tracker", rightCenterTrack);
		SmartDashboard.putBoolean("Right Middle Tracker", rightMiddleTrack);
		SmartDashboard.putBoolean("Right Outer Tracker", rightOuterTrack);
		

		double strafe = Math.pow(Math.abs(Robot.oi.leftJoy.getX()), leftPow) * Math.signum(Robot.oi.leftJoy.getX());
		double forward = Math.pow(Math.abs(Robot.oi.leftJoy.getY()), leftPow) * -Math.signum(Robot.oi.leftJoy.getY());
        double omega = Math.pow(Math.abs(Robot.oi.rightJoy.getX()), rightPow) * Math.signum(Robot.oi.rightJoy.getX()) * OMEGA_SCALE;
       		
        // Add a small deadzone on the joysticks
        if (Math.abs(strafe) < Math.pow(DEADZONE, leftPow)) strafe = 0.0;
		if (Math.abs(forward) < Math.pow(DEADZONE, leftPow)) forward = 0.0;
		if (Math.abs(omega) < Math.pow(DEADZONE, rightPow) * OMEGA_SCALE) omega = 0.0;
		
		
		// If all of the joysticks are in the deadzone, don't update the motors
		// This makes side-to-side strafing much smoother
		if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
			Robot.drive.setDriveLeftFront(0.0);
			Robot.drive.setDriveLeftRear(0.0);
			Robot.drive.setDriveRightFront(0.0);
			Robot.drive.setDriveRightRear(0.0);
			return;
		}
		SmartDashboard.putNumber("Strafe", strafe);
		if (Robot.oi.rightJoy.getTrigger()){
			double orientationError = 0;
			double omegaAngle = 0;

			//Rocket angles below
			if (Robot.oi.rightJoy.getRawButton(2)) {
				if (correctedHeading > 15 && correctedHeading < 45 && targetAngleAquired == false){
					orientationAngle = 29;
					targetAngleAquired = true;
				}
				if (correctedHeading > 135 && correctedHeading < 165 && targetAngleAquired == false){
					orientationAngle = 151;
					targetAngleAquired = true;
				}
				if (correctedHeading > 195 && correctedHeading < 235 && targetAngleAquired == false){
					orientationAngle = 209;
					targetAngleAquired = true;
				}
				if (correctedHeading > 315 && correctedHeading < 345 && targetAngleAquired == false){
					orientationAngle = 331;
					targetAngleAquired = true;
				}
			} else {
				if (correctedHeading > 345 && correctedHeading < 360 || correctedHeading > 0 && correctedHeading < 15 
					&& targetAngleAquired == false){

					orientationAngle = 0;
					targetAngleAquired = true;
				}
				if (correctedHeading > 75 && correctedHeading < 105 && targetAngleAquired == false){
					orientationAngle = 90;
					targetAngleAquired = true;
				}
				if (correctedHeading > 165 && correctedHeading < 195 && targetAngleAquired == false){
					orientationAngle = 180;
					targetAngleAquired = true;
				}
				if (correctedHeading > 255 && correctedHeading < 285 && targetAngleAquired == false){
					orientationAngle = 270;
					targetAngleAquired = true;
				}
			}

			orientationError = orientationAngle - correctedHeading;
			if (Math.abs(orientationError) > 180.0) {
				orientationError -= 360.0 * Math.signum(orientationError);
			}
			if (Math.abs(orientationError) > 2.0){
				omegaAngle = Math.max(Math.min((orientationError / 360) * 0.50, 0.02), -0.02);//start at 0.08
			} else {
				omegaAngle = 0;
			}
			if (targetAngleAquired){
				omega = omegaAngle;
			}

		}
		if(!Robot.oi.rightJoy.getTrigger()){
			targetAngleAquired = false;
		}

		if (!Robot.oi.leftJoy.getTrigger()) {
        	// When the Left Joystick trigger is not pressed, The robot is in Field Centric Mode.
        	// The calculations correct the forward and strafe values for field centric attitude. 
    		
    		// Rotate the velocity vector from the joystick by the difference between our
    		// current orientation and the current origin heading
    		double originCorrection = Math.toRadians(originHeading - RobotMap.navX.getFusedHeading());
    		double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
    		strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);
    		forward = temp;
		}
		
		//Center the robot on the lines using the line tracker sensors
		//This code must come after the Field Centric code as the strafe is being overridden.
		//We want the centering strafe to always act as robot centric to make the 
		//robot move side to side to center itself.
		if (Robot.oi.rightJoy.getTrigger()) {
			
			// if (leftOuterTrack) strafe = -0.22;
			// if (leftMiddleTrack) strafe = -0.2;
			// if (leftCenterTrack) strafe = -0.18;
			// if (rightCenterTrack) strafe = 0.18;
			// if (rightMiddleTrack) strafe = 0.2;
			// if (rightOuterTrack) strafe = 0.22;
			// if (leftCenterTrack && rightCenterTrack) strafe = 0;
			if (leftOuterTrack) strafe = -0.10;
			if (leftOuterTrack && leftMiddleTrack) strafe = -0.9;//-0.21
			if (leftMiddleTrack) strafe = -0.075;
			if (leftMiddleTrack && leftCenterTrack) strafe = -0.075;
			if (leftCenterTrack) strafe = -0.085;
			if (rightCenterTrack) strafe = 0.085;//0.185
			if (rightCenterTrack && rightMiddleTrack) strafe = 0.075;
			if (rightMiddleTrack) strafe = 0.075;//0.195
			if (rightMiddleTrack && rightOuterTrack) strafe = 0.9;
			if (rightOuterTrack) strafe = 0.1;//0.22
			if (leftCenterTrack && rightCenterTrack) strafe = 0;
			
			/*
			if (leftOuterTrack) strafe = -0.22;
			if (leftOuterTrack && leftMiddleTrack) strafe = -0.22;
			if (leftMiddleTrack) strafe = -0.22;
			if (leftMiddleTrack && leftCenterTrack) strafe = -0.2;
			if (leftCenterTrack) strafe = -0.2;
			if (rightCenterTrack) strafe = -0.08;//-0.18
			if (rightMiddleTrack) strafe = 0.08;//0.18
			if (rightOuterTrack && rightMiddleTrack) strafe = 0.2;
			if (rightOuterTrack) strafe = 0.2;
			if (rightCenterTrack && rightMiddleTrack) strafe = 0;
			*/
		}

        Robot.drive.swerveDrive(strafe, forward, omega);
    }

    @Override
	protected boolean isFinished() {
        return false;
    }

    @Override
	protected void end() {
    }

    @Override
	protected void interrupted() {
    	end();
    }

}
