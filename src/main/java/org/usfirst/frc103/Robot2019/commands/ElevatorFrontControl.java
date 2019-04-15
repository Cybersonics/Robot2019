package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class ElevatorFrontControl extends Command {
  public ElevatorFrontControl() {
    requires(Robot.elevatorFront);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.elevatorFront.setElevatorFront(Robot.oi.controller.getY(Hand.kRight));
/*    if (OI.controller.getY(Hand.kRight) > ElevatorFront.DEADZONE) {
      Robot.elevatorFront.setElevatorFront(OI.controller.getY(Hand.kRight));
    }
    else if ((OI.controller.getPOV() == 1) && Robot.elevatorFront.getElevatorFrontEncoder() > 0 && Robot.elevatorFront.getElevatorFrontEncoder()<400) {
      Robot.elevatorFront.setElevatorFrontPosition(500);

    }
    else if ((OI.controller.getPOV()==1) && Robot.elevatorFront.getElevatorFrontEncoder()>450 && Robot.elevatorFront.getElevatorFrontEncoder()<1450){
      Robot.elevatorFront.setElevatorFrontPosition(1500);
    }
    else if ((OI.controller.getPOV()==1) && Robot.elevatorFront.getElevatorFrontEncoder()>1450 && Robot.elevatorFront.getElevatorFrontEncoder()<2000){
      Robot.elevatorFront.setElevatorFrontPosition(2500);
    }
    else if ((OI.controller.getPOV()==5) && Robot.elevatorFront.getElevatorFrontEncoder()<2500 && Robot.elevatorFront.getElevatorFrontEncoder() >1550){
      Robot.elevatorFront.setElevatorFrontPosition(1500);
    }
    else if ((OI.controller.getPOV()==5) && Robot.elevatorFront.getElevatorFrontEncoder()<1500 && Robot.elevatorFront.getElevatorFrontEncoder() >550){
      Robot.elevatorFront.setElevatorFront(1500);
    }
    else if ((OI.controller.getPOV()==5) && Robot.elevatorFront.getElevatorFrontEncoder()<550 && Robot.elevatorFront.getElevatorFrontEncoder() >0){
      Robot.elevatorFront.setElevatorFrontPosition(0);
    }
    else{

    }
    */
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
