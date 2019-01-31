package org.usfirst.frc.team6843.robot.commands;

import org.usfirst.frc.team6843.robot.Robot;
import org.usfirst.frc.team6843.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class DriveForward extends Command {

    private final DriveSubsystem driveSubsystem;

    public DriveForward() {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        requires(this.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        super.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        this.driveSubsystem.arcadeDrive(.75, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (this.driveSubsystem.getLeftPosition() >= 41) {
            return true;
        } else {
            return false;
        }
    }

    // Called once after isFinished returns true
    protected void end() {
    }
}
