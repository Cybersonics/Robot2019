package org.usfirst.frc103.Robot2019;

import static org.usfirst.frc103.Robot2019.RobotMap.driveLeftFront;
import static org.usfirst.frc103.Robot2019.RobotMap.driveLeftRear;
import static org.usfirst.frc103.Robot2019.RobotMap.driveRightFront;
import static org.usfirst.frc103.Robot2019.RobotMap.driveRightRear;

import static org.usfirst.frc103.Robot2019.RobotMap.steerLeftFront;
import static org.usfirst.frc103.Robot2019.RobotMap.steerLeftRear;
import static org.usfirst.frc103.Robot2019.RobotMap.steerRightFront;
import static org.usfirst.frc103.Robot2019.RobotMap.steerRightRear;

import static org.usfirst.frc103.Robot2019.RobotMap.elevatorFront;

import static org.usfirst.frc103.Robot2019.RobotMap.navX;

import java.util.List;
import java.util.stream.Collectors;


import org.usfirst.frc103.Robot2019.commands.CenterDriveForwardSpinSequence;
import org.usfirst.frc103.Robot2019.commands.CenterDriveLeftForwardSpinSequence;
import org.usfirst.frc103.Robot2019.commands.DoNothingAuto;
import org.usfirst.frc103.Robot2019.commands.DriveFieldCentric;
import org.usfirst.frc103.Robot2019.commands.DriveForward;
import org.usfirst.frc103.Robot2019.commands.LeftDriveForwardContinueSequence;
import org.usfirst.frc103.Robot2019.commands.LeftDriveForwardSequence;
import org.usfirst.frc103.Robot2019.commands.LeftDriveForwardSpinSequence;
import org.usfirst.frc103.Robot2019.commands.RightDriveForwardContinueSequence;
import org.usfirst.frc103.Robot2019.commands.RightDriveForwardSequence;
import org.usfirst.frc103.Robot2019.commands.RightDriveForwardSpinSequence;
//import org.usfirst.frc103.Robot2019.commands.VisionAutoSequence;

import org.usfirst.frc103.Robot2019.subsystems.Drive;
import org.usfirst.frc103.Robot2019.subsystems.Pneumatics;
import org.usfirst.frc103.Robot2019.subsystems.Elevators;
import org.usfirst.frc103.Robot2019.subsystems.Intake;

import org.usfirst.frc103.Robot2019.subsystems.Drive;


import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	
	SendableChooser<Command> autonomousChooser;
	Command autonomousCommand;

    public static double zeroHeading;
    
    public static Pneumatics pneumatics = new Pneumatics();
    public static Elevators elevator = new Elevators();
    public static Drive drive = new Drive();
    public static Intake intake = new Intake();


    @Override
	public void robotInit() {
    	RobotMap.init();
    	
   
        
        autonomousChooser = new SendableChooser<Command>();
        
        autonomousChooser.setDefaultOption("Do Nothing", new DoNothingAuto());
        
        SmartDashboard.putData("AutonomousCommands", autonomousChooser);

        zeroHeading = navX.getFusedHeading();
    }

    @Override
	public void disabledInit(){
    	//navX.resetDisplacement();
        //navX.zeroYaw();

    }

    @Override
	public void disabledPeriodic() {
        Scheduler.getInstance().run();
        //RobotMap.drive.encoderReset();
        drive.encoderReset();
        
        updateDashboard();
    }

    @Override
	public void autonomousInit() {
        zeroHeading = navX.getFusedHeading();

        // schedule the autonomous command
    	autonomousCommand = (Command) autonomousChooser.getSelected();
    	if (autonomousCommand != null) {
    		autonomousCommand.start();
    	}
    }

    @Override
	public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        updateDashboard();
 
    }

    @Override
	public void teleopInit() {
        //zeroHeading = navX.getFusedHeading();
    	// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
       
    }

    @Override
	public void teleopPeriodic() {
        Scheduler.getInstance().run();
        updateDashboard();

    	if (RobotMap.leftJoy.getRawButton(10)) zeroHeading = RobotMap.navX.getFusedHeading();
    }

    @Override
    public void testPeriodic() {
        LiveWindow.run();
    }
    
    private void updateDashboard() {
    	SmartDashboard.putNumber("LF Steer Position", steerLeftFront.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("LR Steer Position", steerLeftRear.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("RF Steer Position", steerRightFront.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("RR Steer Position", steerRightRear.getSelectedSensorPosition(0));

    	SmartDashboard.putNumber("LF Drive Position", driveLeftFront.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("LR Drive Position", driveLeftRear.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("RF Drive Position", driveRightFront.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("RR Drive Position", driveRightRear.getSelectedSensorPosition(0));
    	
    	SmartDashboard.putNumber("NavXHeading", navX.getFusedHeading());
    	SmartDashboard.putNumber("NavX Angle", navX.getAngle());
    	SmartDashboard.putNumber("NavXCompass", navX.getCompassHeading());
    	SmartDashboard.putNumber("NavX Yaw", navX.getYaw());
    	//SmartDashboard.putNumber("NavX X Displacement", navX.getDisplacementX());
    	//SmartDashboard.putNumber("NavX Y Displacement", navX.getDisplacementY());
        SmartDashboard.putNumber("ZeroHeading", zeroHeading);
        
        SmartDashboard.putNumber("Front Elevator", elevatorFront.getSelectedSensorPosition(0));

    }
    
}
