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

public class ArcToTarget extends Command {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  public ArcToTarget() {
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
    double lastCompletedRotateTo = RotateTo.getLastCompletedRotateTo();
    double currentHeading = this.driveSubsystem.getGyroAngle();
    double rotationRelativeHeading = currentHeading - lastCompletedRotateTo;
    double angleToTarget = this.visionSubsystem.getTargetAngle();
    double chordLengthInches = this.visionSubsystem.getTargetInchesDistance();
    double chordToTargetAngle = Math.abs(rotationRelativeHeading) - Math.abs(angleToTarget);
    double arcCircleRadiusInches = chordLengthInches / (2.0 * Math.cos(Math.toRadians(90.0 - chordToTargetAngle)));
    double arcConst = 2.0 * Math.PI * (chordToTargetAngle / 360.0);
    double innerArcInches = arcConst * (arcCircleRadiusInches - (DriveSubsystem.LEFT_RIGHT_WHEEL_CENTER_TO_CENTER_INCHES / 2.0));
    double outerArcInches = arcConst * (arcCircleRadiusInches + (DriveSubsystem.LEFT_RIGHT_WHEEL_CENTER_TO_CENTER_INCHES / 2.0));
  
    double leftArcInches = innerArcInches;
    double rightArcInches = outerArcInches;
    if (rotationRelativeHeading < 0.0) {
      leftArcInches = outerArcInches;
      rightArcInches = innerArcInches;
    }

    this.driveSubsystem.startArc(leftArcInches, rightArcInches);
    //this.setTimeout(outerArcInches / 24.0); // 1 sec per 2 feet
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double[] speed = this.driveSubsystem.getArcDriveRates();
    this.driveSubsystem.velocityDrive(speed[0], speed[1]);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return /*this.isTimedOut() ||*/ this.driveSubsystem.isArcOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    this.driveSubsystem.endArc();
    this.driveSubsystem.stop();
  }

}
