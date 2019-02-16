package org.usfirst.frc.team6843.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * To be on release of same button that runs target approach while held.
 */
public class ResetRotatedToTarget extends Command {
  /**
   * Remove all running commands from the scheduler.
   */
  @Override
  protected void initialize() {
    RotateToTarget.setTurnedToTarget(false);
  }

  /**
   * Done right away.
   */
  @Override
  protected boolean isFinished() {
    return true;
  }
}
