package org.usfirst.frc103.Robot2019.subsystems;

import java.util.Arrays;
import java.util.Collections;

import org.usfirst.frc103.Robot2019.RobotMap;
import org.usfirst.frc103.Robot2019.commands.FieldCentricSwerveDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive extends Subsystem {

	private CANSparkMax driveLeftFrontSpark;
	private CANSparkMax driveLeftRearSpark;
	private CANSparkMax driveRightFrontSpark;
	private CANSparkMax driveRightRearSpark;
	// private TalonSRX driveLeftFront;
	// private TalonSRX driveLeftRear;
	// private TalonSRX driveRightFront;
	// private TalonSRX driveRightRear;
	private TalonSRX steerLeftFront;
	private TalonSRX steerLeftRear;
	private TalonSRX steerRightFront;
	private TalonSRX steerRightRear;

	public CANPIDController driveLeftFrontController;
	public CANPIDController driveLeftRearController;
	public CANPIDController driveRightFrontController;
	public CANPIDController driveRightRearController;

	public static final double WHEEL_BASE_LENGTH = 20.5; //28.0 or 20.0;
	public static final double WHEEL_BASE_WIDTH = 24.5; //22.0;
	public static final double ENCODER_COUNT_PER_ROTATION = 1024.0;

	public static final double WHEEL_DIAMETER = 4.0;
	//TODO: increase MAX_SPEED
	public static final double MAX_SPEED = 0.3; //Max speed is 0 to 1 
	public static final double STEER_DEGREES_PER_COUNT = 360.0 / 1024.0;
	public static final double DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.67);
	public static final double DEADZONE = 0.08;
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

	//private static final double DRIVE_P = 7.5, DRIVE_I = 0.0, DRIVE_D = 75.0, DRIVE_F = 1.7, DRIVE_RAMP_RATE = 0.2;
    //private static final int DRIVE_I_ZONE = 0, DRIVE_ALLOWABLE_ERROR = 0, DRIVE_MEASUREMENT_WINDOW = 1;
    //private static final VelocityMeasPeriod DRIVE_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_20Ms;
	private static final double STEER_P = 10.0, STEER_I = 0.02, STEER_D = 0.0;
	private static final int STATUS_FRAME_PERIOD = 5;

	public Drive() {
		driveLeftFrontSpark = new CANSparkMax(10, MotorType.kBrushless);
		driveLeftFrontSpark.restoreFactoryDefaults();
		driveLeftFrontSpark.setIdleMode(IdleMode.kBrake);
		driveLeftFrontSpark.setOpenLoopRampRate(0.125);
		driveLeftFrontSpark.setSmartCurrentLimit(60);
		/*driveLeftFrontController = new CANPIDController(driveLeftFrontSpark);
		driveLeftFrontController.setP(DRIVE_P);
		driveLeftFrontController.setI(DRIVE_I);
		driveLeftFrontController.setD(DRIVE_D);
		driveLeftFrontController.setIZone(DRIVE_I_ZONE);
		driveLeftFrontController*/
		/*driveLeftFront = new TalonSRX(RobotMap.DRIVE_LEFT_FRONT_TALON);
		driveLeftFront.configFactoryDefault();
        driveLeftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        driveLeftFront.setInverted(false);*/
        // driveLeftFront.config_kP(0, DRIVE_P, 0);
        // driveLeftFront.config_kI(0, DRIVE_I, 0);
        // driveLeftFront.config_kD(0, DRIVE_D, 0);
        // driveLeftFront.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        // driveLeftFront.config_kF(0, DRIVE_F, 0);
        // driveLeftFront.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        // driveLeftFront.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        // driveLeftFront.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
		// driveLeftFront.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
		// driveLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD);
		
		
		driveLeftRearSpark = new CANSparkMax(11, MotorType.kBrushless);
		driveLeftRearSpark.restoreFactoryDefaults();
		driveLeftRearSpark.setIdleMode(IdleMode.kBrake);
		driveLeftRearSpark.setOpenLoopRampRate(0.125);
		driveLeftRearSpark.setSmartCurrentLimit(60);
		/*driveLeftRear = new TalonSRX(RobotMap.DRIVE_LEFT_REAR_TALON);
		driveLeftRear.configFactoryDefault();
        driveLeftRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        driveLeftRear.setInverted(false);*/
        // driveLeftRear.config_kP(0, DRIVE_P, 0);
        // driveLeftRear.config_kI(0, DRIVE_I, 0);
        // driveLeftRear.config_kD(0, DRIVE_D, 0);
        // driveLeftRear.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        // driveLeftRear.config_kF(0, DRIVE_F, 0);
        // driveLeftRear.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        // driveLeftRear.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        // driveLeftRear.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        // driveLeftRear.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        // driveLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		driveRightFrontSpark = new CANSparkMax(12, MotorType.kBrushless);
		driveRightFrontSpark.restoreFactoryDefaults();
		driveRightFrontSpark.setIdleMode(IdleMode.kBrake);
		driveRightFrontSpark.setOpenLoopRampRate(0.125);
		driveRightFrontSpark.setSmartCurrentLimit(60);
		/*driveRightFront = new TalonSRX(RobotMap.DRIVE_RIGHT_FRONT_TALON);
		driveRightFront.configFactoryDefault();
        driveRightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        driveRightFront.setInverted(false);*/
        // driveRightFront.config_kP(0, DRIVE_P, 0);
        // driveRightFront.config_kI(0, DRIVE_I, 0);
        // driveRightFront.config_kD(0, DRIVE_D, 0);
        // driveRightFront.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        // driveRightFront.config_kF(0, DRIVE_F, 0);
        // driveRightFront.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        // driveRightFront.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        // driveRightFront.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        // driveRightFront.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        // driveRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		driveRightRearSpark = new CANSparkMax(13, MotorType.kBrushless);
		driveRightRearSpark.restoreFactoryDefaults();
		driveRightRearSpark.setIdleMode(IdleMode.kBrake);
		driveRightRearSpark.setOpenLoopRampRate(0.125);
		driveRightRearSpark.setSmartCurrentLimit(60);
		/*driveRightRear = new TalonSRX(RobotMap.DRIVE_RIGHT_REAR_TALON);
		driveRightRear.configFactoryDefault();
        driveRightRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        driveRightRear.setInverted(false);*/
        // driveRightRear.config_kP(0, DRIVE_P, 0);
        // driveRightRear.config_kI(0, DRIVE_I, 0);
        // driveRightRear.config_kD(0, DRIVE_D, 0);
        // driveRightRear.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        // driveRightRear.config_kF(0, DRIVE_F, 0);
        // driveRightRear.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        // driveRightRear.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        // driveRightRear.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        // driveRightRear.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        // driveRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		steerLeftFront = new TalonSRX(RobotMap.STEER_LEFT_FRONT_TALON);
		steerLeftFront.configFactoryDefault();
        steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerLeftFront.setInverted(false);
        steerLeftFront.config_kP(0, STEER_P, 0);
        steerLeftFront.config_kI(0, STEER_I, 0);
        steerLeftFront.config_kD(0, STEER_D, 0);
        steerLeftFront.config_IntegralZone(0, 100, 0);
        steerLeftFront.configAllowableClosedloopError(0, 5, 0);
        steerLeftFront.setNeutralMode(NeutralMode.Brake);
        steerLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		steerLeftRear = new TalonSRX(RobotMap.STEER_LEFT_REAR_TALON);
		steerLeftRear.configFactoryDefault();
        steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerLeftRear.setInverted(false);
        steerLeftRear.config_kP(0, STEER_P, 0);
        steerLeftRear.config_kI(0, STEER_I, 0);
        steerLeftRear.config_kD(0, STEER_D, 0);
        steerLeftRear.config_IntegralZone(0, 100, 0);
        steerLeftRear.configAllowableClosedloopError(0, 5, 0);
        steerLeftRear.setNeutralMode(NeutralMode.Brake);
        steerLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		steerRightFront = new TalonSRX(RobotMap.STEER_RIGHT_FRONT_TALON);
		steerRightFront.configFactoryDefault();
        steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerRightFront.setInverted(false);
        steerRightFront.config_kP(0, STEER_P, 0);
        steerRightFront.config_kI(0, STEER_I, 0);
        steerRightFront.config_kD(0, STEER_D, 0);
        steerRightFront.config_IntegralZone(0, 100, 0);
        steerRightFront.configAllowableClosedloopError(0, 5, 0);
        steerRightFront.setNeutralMode(NeutralMode.Brake);
        steerRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		steerRightRear = new TalonSRX(RobotMap.STEER_RIGHT_REAR_TALON);
		steerRightRear.configFactoryDefault();
        steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerRightRear.setInverted(false);
        steerRightRear.config_kP(0, STEER_P, 0);
        steerRightRear.config_kI(0, STEER_I, 0);
        steerRightRear.config_kD(0, STEER_D, 0);
        steerRightRear.config_IntegralZone(0, 100, 0);
        steerRightRear.configAllowableClosedloopError(0, 5, 0);
        steerRightRear.setNeutralMode(NeutralMode.Brake);
        steerRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
	}

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
    	// setSwerveModule(steerLeftFront, driveLeftFront, angleLF, speedLF / maxSpeed);
    	// setSwerveModule(steerLeftRear, driveLeftRear, angleLR, speedLR / maxSpeed);
    	// setSwerveModule(steerRightFront, driveRightFront, angleRF, speedRF / maxSpeed);
		// setSwerveModule(steerRightRear, driveRightRear, angleRR, speedRR / maxSpeed);

		setSwerveModule(steerLeftFront, driveLeftFrontSpark, angleLF, speedLF / maxSpeed);
    	setSwerveModule(steerLeftRear, driveLeftRearSpark, angleLR, speedLR / maxSpeed);
    	setSwerveModule(steerRightFront, driveRightFrontSpark, angleRF, speedRF / maxSpeed);
		setSwerveModule(steerRightRear, driveRightRearSpark, angleRR, speedRR / maxSpeed);
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

	//private void setSwerveModule(TalonSRX steer, TalonSRX drive, double angle, double speed) {
	private void setSwerveModule(TalonSRX steer, CANSparkMax drive, double angle, double speed) {
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
		
		//if (Math.abs(speed) <= MAX_SPEED){
    		if (Math.abs(deltaDegrees) > 90.0) {
    			deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
    			speed = -speed;
			}
		//}
		

		double targetPosition = currentPosition + deltaDegrees * ENCODER_COUNT_PER_ROTATION / 360.0;
		steer.set(ControlMode.Position, targetPosition);
		//drive.set(ControlMode.PercentOutput, speed);
		drive.set(speed);

	}

	//get Encoder values

	/*public double getDriveLFEncoder() {
		
		//return driveLeftFront.getSelectedSensorPosition(0);
	}
	
	public double getDriveLREncoder() {
		return driveLeftRear.getSelectedSensorPosition(0);
	}
	
	public double getDriveRFEncoder() {
		return driveRightFront.getSelectedSensorPosition(0);
	}
	
	public double getDriveRREncoder() {
		return driveRightRear.getSelectedSensorPosition(0);
	}*/
	
	public double getSteerLFEncoder() {
		return steerLeftFront.getSelectedSensorPosition(0);
	}
	
	public double getSteerLREncoder() {
		return steerLeftRear.getSelectedSensorPosition(0);
	}
	
	public double getSteerRFEncoder() {
		return steerRightFront.getSelectedSensorPosition(0);
	}
	
	public double getSteerRREncoder() {
		return steerRightRear.getSelectedSensorPosition(0);
	}

	//setting motors
	public void setDriveLeftFront(double speed){
		driveLeftFrontSpark.set(speed);
		//driveLeftFront.set(ControlMode.PercentOutput, speed);
	}

	public void setDriveLeftRear(double speed){
		driveLeftRearSpark.set(speed);
		//driveLeftRear.set(ControlMode.PercentOutput, speed);
	}
	
	public void setDriveRightFront(double speed){
		driveRightFrontSpark.set(speed);
		//driveRightFront.set(ControlMode.PercentOutput, speed);
	}
	
	public void setDriveRightRear(double speed){
		driveRightRearSpark.set(speed);
		//driveRightRear.set(ControlMode.PercentOutput, speed);
	}
	
	public void setSteerLeftFront(double speed){
		steerLeftFront.set(ControlMode.PercentOutput, speed);
	}
	
	public void setSteerLeftRear(double speed){
		steerLeftRear.set(ControlMode.PercentOutput, speed);
	}
	
	public void setSteerRightFront(double speed){
		steerRightFront.set(ControlMode.PercentOutput, speed);
	}
	
	public void setSteerRightRear(double speed){
		steerRightRear.set(ControlMode.PercentOutput, speed);
	}
      public void encoderReset() {
		
		// driveLeftFrontSpark.setSelectedSensorPosition(0, 0, 0);
    	// driveRightFrontSpark.setSelectedSensorPosition(0, 0, 0);
    	// driveLeftRearSpark.setSelectedSensorPosition(0, 0, 0);
		// driveRightRearSpark.setSelectedSensorPosition(0, 0, 0);
		driveLeftFrontSpark.setEncPosition(0);
		driveLeftRearSpark.setEncPosition(0);
		driveRightFrontSpark.setEncPosition(0);
		driveRightRearSpark.setEncPosition(0);
	}
	
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new FieldCentricSwerveDrive());
    }
	
}