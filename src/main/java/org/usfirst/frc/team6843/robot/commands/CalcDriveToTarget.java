/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6843.robot.commands;

import org.usfirst.frc.team6843.robot.Robot;
import org.usfirst.frc.team6843.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team6843.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CalcDriveToTarget extends Command {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  public CalcDriveToTarget() {
    super();
    this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
    this.visionSubsystem = Robot.getInstance().getVisionSubsystem();
    // DO NOT require vision subsystem!
    requires(this.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (!visionSubsystem.isTargetDataGood()) {
      this.cancel();
      return;
    }
    double lastCompletedRotateTo = Robot.getInstance().getLastCompletedRotateTo();
    double currentHeading = this.driveSubsystem.getGyroAngle();
    double rotationRelativeHeading = currentHeading - lastCompletedRotateTo; // R
    double absRotationRelativeHeading = Math.abs(rotationRelativeHeading);
    double angleToTarget = this.visionSubsystem.getTargetAngle(); // T
    double absAngleToTarget = Math.abs(angleToTarget);
    double firstHeadingSign = angleToTarget >= 0.0 ? 1.0 : -1.0;
    if (((rotationRelativeHeading > 0.0) && (angleToTarget < 0.0))
        || ((rotationRelativeHeading < 0.0) && (angleToTarget > 0.0))) {
      firstHeadingSign = rotationRelativeHeading >= 0.0 ? 1.0 : -1.0;
    }
    double firstHeading = absRotationRelativeHeading + absAngleToTarget;
    //firstHeading = Math.max(firstHeading, 15.0);
    if (((firstHeading > 0.0) && (firstHeadingSign < 0.0))
        || ((firstHeading < 0.0) && (firstHeadingSign > 0.0))) {
      firstHeading = firstHeading * -1.0;
    }
    firstHeading += lastCompletedRotateTo;
    double targetDistInches = this.visionSubsystem.getTargetInchesDistance();

    CommandGroup cg = new CommandGroup();
    cg.addSequential(new PivotTo(firstHeading));
    cg.addSequential(new DriveTo(targetDistInches * 0.75));
    cg.addSequential(new PivotTo(lastCompletedRotateTo));
    // cg.addSequential(new DriveToTarget());

    cg.start();
    cg.close();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }
}
