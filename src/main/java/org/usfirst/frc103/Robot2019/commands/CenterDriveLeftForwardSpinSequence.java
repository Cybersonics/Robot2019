package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CenterDriveLeftForwardSpinSequence extends CommandGroup {
	
	public CenterDriveLeftForwardSpinSequence() {
		requires(RobotMap.drive);
		
		
	//	addSequential(new VisionPlaceGear(0.0));
	//	addSequential(new VisionLeaveGear());
	
	//before vision
		//addSequential(new DriveForward(-107.0, 0));
		//addSequential(new ShootAuto(3550));
		addSequential(new DriveForward(-80.0, 0));
		
	}
	
}
