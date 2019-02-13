package org.usfirst.frc103.Robot2019.subsystems;

import static org.usfirst.frc103.Robot2019.RobotMap.driveLeftFront;
import static org.usfirst.frc103.Robot2019.RobotMap.driveLeftRear;
import static org.usfirst.frc103.Robot2019.RobotMap.driveRightFront;
import static org.usfirst.frc103.Robot2019.RobotMap.driveRightRear;
import static org.usfirst.frc103.Robot2019.RobotMap.steerLeftFront;
import static org.usfirst.frc103.Robot2019.RobotMap.steerLeftRear;
import static org.usfirst.frc103.Robot2019.RobotMap.steerRightFront;
import static org.usfirst.frc103.Robot2019.RobotMap.steerRightRear;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.Collections;

import org.usfirst.frc103.Robot2019.commands.FieldCentricSwerveDrive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Drive extends Subsystem {

	public static final double WHEEL_BASE_LENGTH = 28.0;
	public static final double WHEEL_BASE_WIDTH = 22.0;
	public static final double ENCODER_COUNT_PER_ROTATION = 1024.0;

	//XXX: FIX THIS BEFORE COMPETITION, WHEEL DIAMETER SHOULD BE 4" maybe
	public static final double WHEEL_DIAMETER = 3.95;
	public static final double MAX_SPEED = 0.3; //Max speed is 0 to 1 
	public static final double STEER_DEGREES_PER_COUNT = 360.0 / 1024.0;
	public static final double DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.67);
	public static final double DEADZONE = 0.08;
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

	public void swerveDrive(double strafe, double forward, double omega) {
        double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
        double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);
        
        // Compute the constants used later for calculating speeds and angles
        double A = strafe - omegaL2;
        double B = strafe + omegaL2;
        double C = forward - omegaW2;
        double D = forward + omegaW2;
        
        // Compute the drive motor speeds
        double speedLF = speed(B, D);
        double speedLR = speed(A, D);
        double speedRF = speed(B, C);
        double speedRR = speed(A, C);
        
		// ... and angles for the steering motors 
		// When drives are calibrated for zero position on encoders they are at 90 degrees
		// to the front of the robot. Subtract and add 90 degrees to steering calculation to offset
		// for initial position/calibration of drives.

		double angleLF = angle(B, D) - 90;
    	double angleLR = angle(A, D) + 90;
    	double angleRF = angle(B, C) - 90;
    	double angleRR = angle(A, C) + 90;
    	// Compute the maximum speed so that we can scale all the speeds to the range [0, 1]
    	double maxSpeed = Collections.max(Arrays.asList(speedLF, speedLR, speedRF, speedRR, 1.0));

    	// Set each swerve module, scaling the drive speeds by the maximum speed
    	setSwerveModule(steerLeftFront, driveLeftFront, angleLF, speedLF / maxSpeed);
    	setSwerveModule(steerLeftRear, driveLeftRear, angleLR, speedLR / maxSpeed);
    	setSwerveModule(steerRightFront, driveRightFront, angleRF, speedRF / maxSpeed);
		setSwerveModule(steerRightRear, driveRightRear, angleRR, speedRR / maxSpeed);
		
		SmartDashboard.putNumber("LF Steer Angle", angleLF);
    	SmartDashboard.putNumber("LR Steer Angle", angleLR);
    	SmartDashboard.putNumber("RF Steer Angle", angleRF);
    	SmartDashboard.putNumber("RR Steer Angle", angleRR);

    	SmartDashboard.putNumber("LF Drive Speed", speedLF);
    	SmartDashboard.putNumber("LR Drive Speed", speedLR);
    	SmartDashboard.putNumber("RF Drive Speed", speedRF);
    	SmartDashboard.putNumber("RR Drive Speed", speedRR);
	}
	
	private double speed(double val1, double val2){
    	return Math.hypot(val1, val2);
    }
    
    private double angle(double val1, double val2){
    	return Math.toDegrees(Math.atan2(val1, val2));
    }
	
	/*
	private void setSwerveModule(TalonSRX steer, TalonSRX drive, double angle, double speed) {
		// Get the current angle and speed for the module
		double currentAngle = steer.getSelectedSensorPosition(0) * STEER_DEGREES_PER_COUNT;
		double currentSpeed = drive.getSelectedSensorVelocity(0) * 10.0 * DRIVE_INCHES_PER_COUNT;
		
		// Calculate the number of degrees to turn assuming that speed is not reversed
		double angleDelta = Math.IEEEremainder(angle - currentAngle, 360.0);
		// Calculate the corresponding change in speed required
		double speedDifference = Math.abs(speed - currentSpeed);

		// Calculate the angle that requires the least amount of turning by allowing speed reversal
		double shortestAngleDelta = Math.IEEEremainder(angleDelta, 180.0);
		// If the previous calculation flipped the direction to turn, then the speed must be reversed
		double shortestSpeed = Math.signum(angleDelta) * Math.signum(shortestAngleDelta) * speed;
		// Calculate the change in speed when speed reversal is allowed
		double shortestSpeedDifference = Math.abs(shortestSpeed - currentSpeed);

		// If the change in speed required when using the angle that requires the least amount of turning is below the
		// speed reversal threshold, then always use that angle and corresponding speed
		// If the change in speed is above the reversal speed threshold, prefer the turning direction that results in
		// the least change in speed
		if (shortestSpeedDifference <= MAX_REVERSIBLE_SPEED_DIFFERENCE || shortestSpeedDifference <= speedDifference) {
			angleDelta = shortestAngleDelta;
			speed = shortestSpeed;
		}

		// Set the steering motor position and drive motor output accordingly
		steer.set(ControlMode.Position, (currentAngle + angleDelta) / STEER_DEGREES_PER_COUNT);
		drive.set(ControlMode.PercentOutput, speed / MAX_SPEED);
	}

	*/	


	
	private void setSwerveModule(TalonSRX steer, TalonSRX drive, double angle, double speed) {
    	double currentPosition = steer.getSelectedSensorPosition(0);
    	double currentAngle = (currentPosition * 360.0 / ENCODER_COUNT_PER_ROTATION) % 360.0;
    	// The angle from the encoder is in the range [0, 360], but the swerve computations
    	// return angles in the range [-180, 180], so transform the encoder angle to this range
    	if (currentAngle > 180.0) {
    		currentAngle -= 360.0;
    	}
    	// TODO: Properly invert the steering motors so this isn't necessary
    	// This is because the steering encoders are inverted
    	double targetAngle = -angle;
    	double deltaDegrees = targetAngle - currentAngle;
    	// If we need to turn more than 180 degrees, it's faster to turn in the opposite direction
    	if (Math.abs(deltaDegrees) > 180.0) {
    		deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    	}
    	// If we need to turn more than 90 degrees, we can reverse the wheel direction instead and
		// only rotate by the complement
		/*
		if (Math.abs(speed) <= MAX_SPEED){
    		if (Math.abs(deltaDegrees) > 90.0) {
    			deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
    			speed = -speed;
			}
		}
		*/

		double targetPosition = currentPosition + deltaDegrees * ENCODER_COUNT_PER_ROTATION / 360.0;
		steer.set(ControlMode.Position, targetPosition);
		drive.set(ControlMode.PercentOutput, speed);

	}
	
    
    public void encoderReset() {
		
		driveLeftFront.setSelectedSensorPosition(0, 0, 0);
    	driveRightFront.setSelectedSensorPosition(0, 0, 0);
    	driveLeftRear.setSelectedSensorPosition(0, 0, 0);
    	driveRightRear.setSelectedSensorPosition(0, 0, 0);
    }
	
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new FieldCentricSwerveDrive());
    }
	
}
