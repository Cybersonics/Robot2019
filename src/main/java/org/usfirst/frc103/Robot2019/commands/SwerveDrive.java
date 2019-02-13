package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class SwerveDrive extends Command {

    public SwerveDrive() {
        requires(RobotMap.drive);
    }

    @Override
	protected void execute() {
		double vX = RobotMap.leftJoy.getX(), vY = -RobotMap.leftJoy.getY();
        double omega = RobotMap.rightJoy.getX() / 30.0;
        RobotMap.drive.swerveDrive(vX, vY, omega);
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
