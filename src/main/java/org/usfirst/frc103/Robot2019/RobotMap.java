package org.usfirst.frc103.Robot2019;

import org.usfirst.frc103.Robot2019.subsystems.RangeFinder;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import edu.wpi.first.wpilibj.CameraServer;
//import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import org.usfirst.frc103.Robot2019.subsystems.Drive;
import org.usfirst.frc103.Robot2019.subsystems.Pneumatics;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
    public static TalonSRX driveLeftFront;
    public static TalonSRX driveLeftRear;
    public static TalonSRX driveRightFront;
    public static TalonSRX driveRightRear;
    public static TalonSRX steerLeftFront;
    public static TalonSRX steerLeftRear;
    public static TalonSRX steerRightFront;
    public static TalonSRX steerRightRear;
    
    public static AHRS navX;
 
    public static Ultrasonic ultrasonic;

    public static Pneumatics pneumatics;

    public static DoubleSolenoid hatchPanelSolenoid;
    public static DoubleSolenoid armLockSolenoid;


    public static Drive drive;

    public static Joystick leftJoy;
    public static Joystick rightJoy;
    public static XboxController controller;

    private static final double DRIVE_P = 7.5, DRIVE_I = 0.0, DRIVE_D = 75.0, DRIVE_F = 1.7, DRIVE_RAMP_RATE = 0.2;
    private static final int DRIVE_I_ZONE = 0, DRIVE_ALLOWABLE_ERROR = 0, DRIVE_MEASUREMENT_WINDOW = 1;
    private static final VelocityMeasPeriod DRIVE_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_20Ms;
    private static final double STEER_P = 10.0, STEER_I = 0.02, STEER_D = 0.0;
    private static final int STATUS_FRAME_PERIOD = 5;

    public static void init() {
        driveLeftFront = new TalonSRX(10);
        driveLeftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        driveLeftFront.setInverted(false);

        driveLeftFront.config_kP(0, DRIVE_P, 0);
        driveLeftFront.config_kI(0, DRIVE_I, 0);
        driveLeftFront.config_kD(0, DRIVE_D, 0);
        driveLeftFront.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveLeftFront.config_kF(0, DRIVE_F, 0);
        driveLeftFront.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveLeftFront.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveLeftFront.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveLeftFront.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        
        driveLeftRear = new TalonSRX(11);
        driveLeftRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        driveLeftRear.setInverted(false);

        driveLeftRear.config_kP(0, DRIVE_P, 0);
        driveLeftRear.config_kI(0, DRIVE_I, 0);
        driveLeftRear.config_kD(0, DRIVE_D, 0);
        driveLeftRear.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveLeftRear.config_kF(0, DRIVE_F, 0);
        driveLeftRear.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveLeftRear.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveLeftRear.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveLeftRear.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        driveRightFront = new TalonSRX(12);
        driveRightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        driveRightFront.setInverted(false);

		//XXX: REMOVE THIS BEFORE COMPETITION, SENSOR PHASE SHOULD NOT BE INVERTED
        //driveRightFront.setSensorPhase(true);
        driveRightFront.config_kP(0, DRIVE_P, 0);
        driveRightFront.config_kI(0, DRIVE_I, 0);
        driveRightFront.config_kD(0, DRIVE_D, 0);
        driveRightFront.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveRightFront.config_kF(0, DRIVE_F, 0);
        driveRightFront.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveRightFront.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveRightFront.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveRightFront.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        driveRightRear = new TalonSRX(13);
        driveRightRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        driveRightRear.setInverted(false);

		//XXX: REMOVE THIS BEFORE COMPETITION, SENSOR PHASE SHOULD NOT BE INVERTED
        //driveRightRear.setSensorPhase(true);
        driveRightRear.config_kP(0, DRIVE_P, 0);
        driveRightRear.config_kI(0, DRIVE_I, 0);
        driveRightRear.config_kD(0, DRIVE_D, 0);
        driveRightRear.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveRightRear.config_kF(0, DRIVE_F, 0);
        driveRightRear.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveRightRear.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveRightRear.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveRightRear.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerLeftFront = new TalonSRX(16);
        steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

        steerLeftFront.setInverted(false);

        steerLeftFront.config_kP(0, STEER_P, 0);
        steerLeftFront.config_kI(0, STEER_I, 0);
        steerLeftFront.config_kD(0, STEER_D, 0);
        steerLeftFront.config_IntegralZone(0, 100, 0);
        steerLeftFront.configAllowableClosedloopError(0, 5, 0);
        steerLeftFront.setNeutralMode(NeutralMode.Brake);
        steerLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerLeftRear = new TalonSRX(17);
        steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

        steerLeftRear.setInverted(false);

        steerLeftRear.config_kP(0, STEER_P, 0);
        steerLeftRear.config_kI(0, STEER_I, 0);
        steerLeftRear.config_kD(0, STEER_D, 0);
        steerLeftRear.config_IntegralZone(0, 100, 0);
        steerLeftRear.configAllowableClosedloopError(0, 5, 0);
        steerLeftRear.setNeutralMode(NeutralMode.Brake);
        steerLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerRightFront = new TalonSRX(18);
        steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

        steerRightFront.setInverted(false);

        steerRightFront.config_kP(0, STEER_P, 0);
        steerRightFront.config_kI(0, STEER_I, 0);
        steerRightFront.config_kD(0, STEER_D, 0);
        steerRightFront.config_IntegralZone(0, 100, 0);
        steerRightFront.configAllowableClosedloopError(0, 5, 0);
        steerRightFront.setNeutralMode(NeutralMode.Brake);
        steerRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerRightRear = new TalonSRX(19);
        steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

        steerRightRear.setInverted(false);

        steerRightRear.config_kP(0, STEER_P, 0);
        steerRightRear.config_kI(0, STEER_I, 0);
        steerRightRear.config_kD(0, STEER_D, 0);
        steerRightRear.config_IntegralZone(0, 100, 0);
        steerRightRear.configAllowableClosedloopError(0, 5, 0);
        steerRightRear.setNeutralMode(NeutralMode.Brake);
        steerRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        hatchPanelSolenoid = new DoubleSolenoid(0, 1);
        armLockSolenoid = new DoubleSolenoid(2, 3);
        pneumatics = new Pneumatics();

        
        drive = new Drive();

        controller = new XboxController(2);
        leftJoy = new Joystick(0);
        rightJoy = new Joystick(1);
        
        navX = new AHRS(SPI.Port.kMXP);
      
        //ultrasonic = new Ultrasonic(8, 9);
        //RangeFinder.start();

        //CameraServer.getInstance().startAutomaticCapture();
    }
    
}
