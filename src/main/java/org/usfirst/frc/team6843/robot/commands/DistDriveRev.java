package org.usfirst.frc.team6843.robot.commands;

import org.usfirst.frc.team6843.robot.OI;
import org.usfirst.frc.team6843.robot.Robot;
import org.usfirst.frc.team6843.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DistDriveRev extends Command {

	protected final DriveSubsystem driveSubsystem;
	protected OI oi;
	
    public DistDriveRev() {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        requires(this.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		super.initialize();
		this.oi = Robot.getInstance().getOI();
		this.driveSubsystem.clearEncoders();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() { 
    		this.driveSubsystem.velocityDrive(-1000, 1000);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    			/*if(this.driveSubsystem.getLeftPosition() >= 15 && this.driveSubsystem.getRightPosition() <= -15) {
    				return true;
    			} else {
                    return false;*/
                return false;
    			
    }

    // Called once after isFinished returns true
    protected void end() {
    	this.driveSubsystem.stop();
    }
    
}
