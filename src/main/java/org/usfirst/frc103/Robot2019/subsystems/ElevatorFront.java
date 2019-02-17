package org.usfirst.frc103.Robot2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc103.Robot2019.RobotMap;
import org.usfirst.frc103.Robot2019.commands.ElevatorFrontControl;

public class ElevatorFront extends Subsystem {
  private TalonSRX elevatorFront;

  public static final double DEADZONE = 0.05;
  private static final double ELEVATOR_P = 10.0, ELEVATOR_I = 0.02, ELEVATOR_D = 0.0;
  private static final int STATUS_FRAME_PERIOD = 5;

  public ElevatorFront() {
    elevatorFront = new TalonSRX(RobotMap.ELEVATOR_FRONT_TALON);
    elevatorFront.configSelectedFeedbackSensor(FeedbackDevice.Analog);
/*    elevatorFront.config_kP(0, ELEVATOR_P, 0);
    elevatorFront.config_kI(0, ELEVATOR_I, 0);
    elevatorFront.config_kD(0, ELEVATOR_D, 0);
    elevatorFront.config_IntegralZone(0, 100, 0);
    elevatorFront.configAllowableClosedloopError(0, 5, 0);
*/    elevatorFront.setNeutralMode(NeutralMode.Brake);
    elevatorFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
  }

  public void setElevatorFront(double frontLift) {
    if (Math.abs(frontLift) < DEADZONE){
      elevatorFront.set(ControlMode.PercentOutput, 0.0);
    } else {
      elevatorFront.set(ControlMode.PercentOutput, -frontLift);
    }
  }

  public double getElevatorFrontEncoder(){
    return elevatorFront.getSelectedSensorPosition(0);
  }

  public void setElevatorFrontPosition(double position) {
    elevatorFront.set(ControlMode.Position, position);
  }

  //XXX: Not sure if this is right
  public void zeroElevatorFrontPosition() {
    elevatorFront.set(ControlMode.Position, 0.0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ElevatorFrontControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
