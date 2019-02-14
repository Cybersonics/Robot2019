package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class PneumaticControl extends Command {
  boolean bPressed;
  
  public PneumaticControl() {
    requires(Robot.pneumatics);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  @Override
  protected void initialize() {
    bPressed = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (RobotMap.controller.getAButtonPressed()){
      RobotMap.hatchPanelSolenoid.set(Value.kForward);
    } else if(RobotMap.controller.getAButtonReleased()) {
      RobotMap.hatchPanelSolenoid.set(Value.kReverse);
    }

    /*
    if (RobotMap.controller.getBButtonPressed()){
      RobotMap.armLockSolenoid.set(Value.kReverse);
    } else if(RobotMap.controller.getBButtonReleased()) {
      RobotMap.armLockSolenoid.set(Value.kForward);
    }
    */

    if(RobotMap.controller.getBButton() && !bPressed){
      RobotMap.armLockSolenoid.set(Value.kForward);
      if(RobotMap.controller.getBButtonReleased()){
        bPressed = true;
      }
    }
    else if(RobotMap.controller.getBButton() && bPressed){
      RobotMap.armLockSolenoid.set(Value.kReverse);
      if(RobotMap.controller.getBButtonReleased()){
        bPressed = false;
      }
    }
    else {
      RobotMap.armLockSolenoid.set(Value.kOff);
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
