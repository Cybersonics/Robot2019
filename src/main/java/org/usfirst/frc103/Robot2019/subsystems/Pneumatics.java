package org.usfirst.frc103.Robot2019.subsystems;


import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc103.Robot2019.commands.PneumaticControl;

public class Pneumatics extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new PneumaticControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
