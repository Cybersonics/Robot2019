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
		SmartDashboard.putNumber("OriginHeading", originHeading);
		SmartDashboard.putNumber("OriginCorrection", originCorr);
		SmartDashboard.putNumber("OriginOffset", originOffset);
		SmartDashboard.putNumber("CorrectedHeading", correctedHeading);
    		
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
		
		if (Robot.oi.rightJoy.getTrigger()){
			double orientationError = 0;
			double omegaAngle = 0;
			boolean targetAngleAquired = false;
			if (correctedHeading > 75 && correctedHeading < 105){
				orientationError = 90 - correctedHeading;
				targetAngleAquired = true;
			}
			if (correctedHeading > 165 && correctedHeading < 195){
				orientationError = 180 - correctedHeading;
				targetAngleAquired = true;
			}
			if (correctedHeading > 255 && correctedHeading < 285){
				orientationError = 270 - correctedHeading;
				targetAngleAquired = true;
			}
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
