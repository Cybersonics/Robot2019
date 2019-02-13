package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class LeftDriveForwardSequence extends CommandGroup {
	
	public LeftDriveForwardSequence() {
		requires(RobotMap.drive);

		addSequential(new DriveForward(60.0, 4600));
	//	addSequential(new VisionPlaceGear(60.0));
	//	addSequential(new VisionLeaveGear());
	}
	
}
