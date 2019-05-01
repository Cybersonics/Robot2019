package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc103.Robot2019.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class ElevatorRearControl extends Command {
  boolean rearLiftLocked;
    
  public ElevatorRearControl() {
    requires(Robot.elevatorRear);
  }

  @Override
  protected void initialize() {
    rearLiftLocked = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.elevatorRear.setElevatorRear(Robot.oi.controller.getY(Hand.kLeft), rearLiftLocked);
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
