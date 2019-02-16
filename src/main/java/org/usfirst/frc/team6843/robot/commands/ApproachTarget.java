/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6843.robot.commands;

import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class ApproachTarget extends ConditionalCommand {
  /**
   * First runs rotate and then when that is done, runs drive.
   */
  public ApproachTarget() {
    super(new DriveToTarget(), new RotateToTarget());
  }

  public boolean condition() {
    return RotateToTarget.isTurnedToTarget();
  }
}
