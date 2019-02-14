package org.usfirst.frc103.Robot2019.subsystems;


import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc103.Robot2019.commands.IntakeControl;

public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  protected void initDefaultCommand() {
    
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeControl());
  }
}
