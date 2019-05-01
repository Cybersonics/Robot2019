package org.usfirst.frc103.Robot2019;

import org.usfirst.frc103.Robot2019.subsystems.RangeFinder;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;
//import edu.wpi.first.wpilibj.CameraServer;
//import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cameraserver.CameraServer;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    public static AHRS navX; 
    public static Ultrasonic ultrasonic;
    public static DigitalInput leftOuterLineTracker;
    public static DigitalInput leftMiddleLineTracker;
    public static DigitalInput leftCenterLineTracker;
    public static DigitalInput rightCenterLineTracker;
    public static DigitalInput rightMiddleLineTracker;
    public static DigitalInput rightOuterLineTracker;

    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int CONTROLLER = 2;

    public static final int DRIVE_LEFT_FRONT_SPARK = 10;
    public static final int DRIVE_LEFT_REAR_SPARK = 11;
    public static final int DRIVE_RIGHT_FRONT_SPARK = 12;
    public static final int DRIVE_RIGHT_REAR_SPARK = 13;
    // public static final int DRIVE_LEFT_FRONT_TALON = 10;
    // public static final int D RIVE_LEFT_REAR_TALON = 11;
    // public static final int DRIVE_RIGHT_FRONT_TALON = 12;
    // public static final int DRIVE_RIGHT_REAR_TALON = 13;
    public static final int STEER_LEFT_FRONT_TALON = 16;
    public static final int STEER_LEFT_REAR_TALON = 17;
    public static final int STEER_RIGHT_FRONT_TALON = 18;
    public static final int STEER_RIGHT_REAR_TALON = 19;

    public static final int ELEVATOR_FRONT_TALON = 20;
    public static final int ELEVATOR_REAR_TALON = 21;

    public static final int INTAKE_TALON = 22;
    public static final int ARM_TALON = 23;

    public static final int HATCH_PANEL_SOLENOID_FORWARD = 0;
    public static final int HATCH_PANEL_SOLENOID_REVERSE = 1;
    public static final int ARM_LOCK_SOLENOID_FORWARD = 2;
    public static final int ARM_LOCK_SOLENOID_REVERSE = 3;
    public static final int ELEVATOR_REAR_LOCK_FORWARD = 4;
    public static final int ELEVATOR_REAR_LOCK_REVERSE = 5;

    public static void init() {

        
        navX = new AHRS(SPI.Port.kMXP);
        leftOuterLineTracker = new DigitalInput(10);
        leftMiddleLineTracker = new DigitalInput(11);
        leftCenterLineTracker = new DigitalInput(12);
        rightCenterLineTracker = new DigitalInput(13);
        rightMiddleLineTracker = new DigitalInput(9);
        rightOuterLineTracker = new DigitalInput(8);

        //ultrasonic = new Ultrasonic(8, 9);
        //RangeFinder.start();

        CameraServer.getInstance().startAutomaticCapture();

    }
    
}
