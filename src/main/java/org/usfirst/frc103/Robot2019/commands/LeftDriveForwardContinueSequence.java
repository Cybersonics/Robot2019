package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class LeftDriveForwardContinueSequence extends CommandGroup {
	
	public LeftDriveForwardContinueSequence() {
		requires(RobotMap.drive);
	

		addSequential(new LeftDriveForwardSequence());
		addSequential(new DriveFieldCentric(0.0, 10000.0, 0.0));
	}

}
