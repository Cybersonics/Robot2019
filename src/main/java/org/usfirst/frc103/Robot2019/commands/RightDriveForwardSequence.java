package org.usfirst.frc103.Robot2019.commands;


import org.usfirst.frc103.Robot2019.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class RightDriveForwardSequence extends CommandGroup {
	
	public RightDriveForwardSequence() {
		requires(Robot.drive);
		
		addSequential(new DriveForward(-60.0, 4500)); //Distance was 3800
	//	addSequential(new VisionPlaceGear(-60.0));
	//	addSequential(new VisionLeaveGear());
	}
	
}
