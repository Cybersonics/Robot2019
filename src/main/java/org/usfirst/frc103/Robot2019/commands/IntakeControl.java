package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc103.Robot2019.Robot;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class IntakeControl extends Command {
  
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
    if (Robot.oi.leftJoy.getRawButton(2)) {
      //Robot.intake.climbIntake();
      Robot.intake.driverIntakeIn();
    } else if (Robot.oi.rightJoy.getRawButton(2)) {
      Robot.intake.driverIntakeOut();
    } else {
      Robot.intake.intakeRun(Robot.oi.controller.getTriggerAxis(Hand.kLeft), Robot.oi.controller.getTriggerAxis(Hand.kRight));
    }
    if (Robot.oi.leftJoy.getRawButton(3)) {
      Robot.intake.driverBallIntakeIn();
    }

    if (Robot.oi.controller.getPOV() == 180) {
      Robot.intake.operatorLowIntakeIn();
    }

    if (Robot.oi.controller.getPOV() == 270) {
      Robot.intake.operatorMediumIntakeIn();
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
