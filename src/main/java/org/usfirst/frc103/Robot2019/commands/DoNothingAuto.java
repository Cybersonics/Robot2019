package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;

public class DoNothingAuto extends Command {
	
	public DoNothingAuto() {
		
	}
	
	@Override
	protected void execute() {
		
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
    	end();
    }

}
