package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class SwerveDrive extends Command {

    public SwerveDrive() {
        requires(Robot.drive);
    }

    @Override
	protected void execute() {
		double vX = Robot.oi.leftJoy.getX(), vY = -Robot.oi.leftJoy.getY();
        double omega = Robot.oi.rightJoy.getX() / 30.0;
        Robot.drive.swerveDrive(vX, vY, omega);
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
