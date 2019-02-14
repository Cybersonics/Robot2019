package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;


public class IntakeControl extends Command {
  
  public static final double DEADZONE = 0.05;

  
  public IntakeControl() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
  }

  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    boolean intakeIn = RobotMap.controller.getBumper(Hand.kLeft);
    boolean intakeOut = RobotMap.controller.getBumper(Hand.kRight);
    double armPositionUp = RobotMap.controller.getTriggerAxis(Hand.kLeft);
    double armPositionDown = RobotMap.controller.getTriggerAxis(Hand.kRight);

    if ((armPositionUp > 0 && armPositionDown > 0) || (armPositionUp < DEADZONE && armPositionDown < DEADZONE)){
      RobotMap.armMotor.set(ControlMode.PercentOutput, 0.0);
    } else {
      if (armPositionUp > 0){
        RobotMap.armMotor.set(ControlMode.PercentOutput, -armPositionUp);
      }
      if (armPositionDown > 0){
        RobotMap.armMotor.set(ControlMode.PercentOutput, armPositionDown);
      }
  }

  if ((intakeIn && intakeOut) || (!intakeIn && !intakeOut)){
    RobotMap.intakeMotor.set(ControlMode.PercentOutput, 0.0);
  } else {
    if (intakeIn){
      RobotMap.intakeMotor.set(ControlMode.PercentOutput, 0.5);
    }
    if (intakeOut){
      RobotMap.intakeMotor.set(ControlMode.PercentOutput, -0.6);
    }


  }
}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
