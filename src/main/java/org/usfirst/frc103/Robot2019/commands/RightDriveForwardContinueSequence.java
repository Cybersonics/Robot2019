package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.RobotMap;
import org.usfirst.frc103.Robot2019.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class RightDriveForwardContinueSequence extends CommandGroup {
	
	public RightDriveForwardContinueSequence() {
		requires(Robot.drive);
		

		addSequential(new RightDriveForwardSequence());
		addSequential(new DriveFieldCentric(0.0, 10000.0, 0.0));
	}

}
