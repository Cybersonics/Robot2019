package org.usfirst.frc103.Robot2019.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc103.Robot2019.commands.ElevatorRearControl;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc103.Robot2019.RobotMap;

public class ElevatorRear extends Subsystem {
  private TalonSRX elevatorRear;
  private DoubleSolenoid elevatorRearLock;
  private static final int MAX_AMPS = 5;
  private static final int MAX_AMPS_DURATION = 0;

  public static final double DEADZONE = 0.05;

  public ElevatorRear() {
    elevatorRear = new TalonSRX(RobotMap.ELEVATOR_REAR_TALON);
    elevatorRear.configFactoryDefault();
    elevatorRear.configPeakCurrentLimit(MAX_AMPS, MAX_AMPS_DURATION);
    elevatorRear.configPeakCurrentDuration(0, 0);
    elevatorRear.configContinuousCurrentLimit(MAX_AMPS);

    elevatorRearLock = new DoubleSolenoid(RobotMap.ELEVATOR_REAR_LOCK_FORWARD, RobotMap.ELEVATOR_REAR_LOCK_REVERSE);
  }

//may need to change lock to toggle
  public void setElevatorRear(double rearLift, boolean locked) {
    if (Math.abs(rearLift) < DEADZONE){
      elevatorRear.set(ControlMode.PercentOutput, 0.0);
      if (!locked) {
        elevatorRearLock.set(Value.kReverse);
        locked = true;
      }
    } else {
      elevatorRearLock.set(Value.kForward);
      locked = false;
      
      if (rearLift >= 0 ) {
        elevatorRear.enableCurrentLimit(false);
      } else {
        elevatorRear.enableCurrentLimit(true);
      }

      elevatorRear.set(ControlMode.PercentOutput, rearLift);
    }
  }

  @Override
  protected void initDefaultCommand() {    
    setDefaultCommand(new ElevatorRearControl());
  }
}
