/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6843.robot.commands;

import org.usfirst.frc.team6843.robot.Robot;
import org.usfirst.frc.team6843.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class RotateRight extends Command {

  protected final DriveSubsystem driveSubsystem;

  public RotateRight() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        requires(this.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    this.driveSubsystem.startTurn(driveSubsystem.getGyroAngle()+ 90.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double speed = 1000.0 * this.driveSubsystem.getGyroTurnRate();
    //if(speed >= 0){
    //  speed += 200;
    //}
    //else{
    //  speed -= 200;
    //}
    this.driveSubsystem.encoderTest(speed, speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return this.driveSubsystem.isTurnOnTarget();
  
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    this.driveSubsystem.endTurn();
    this.driveSubsystem.stop();
  }

}
