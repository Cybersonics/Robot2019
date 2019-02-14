package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;


public class ElevatorControl extends Command {
  
  public static final double DEADZONE = 0.05;

  
  public ElevatorControl() {
    // Use requires() here to declare subsystem dependencies
    //requires(RobotMap.elevator);
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double frontLift = RobotMap.controller.getY(Hand.kRight);
    if (Math.abs(frontLift) < DEADZONE){
      RobotMap.elevatorFront.set(ControlMode.PercentOutput, 0.0);
    } else {
      RobotMap.elevatorFront.set(ControlMode.PercentOutput, -frontLift);
    }

    double rearLift = RobotMap.controller.getY(Hand.kLeft);
    if (Math.abs(rearLift) < DEADZONE){
      RobotMap.elevatorRear.set(ControlMode.PercentOutput, 0.0);
    } else {
      RobotMap.elevatorRear.set(ControlMode.PercentOutput, rearLift);
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
