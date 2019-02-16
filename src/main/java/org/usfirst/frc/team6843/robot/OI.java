/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6843.robot;

import org.usfirst.frc.team6843.robot.commands.ApproachTarget;
import org.usfirst.frc.team6843.robot.commands.KillAll;
import org.usfirst.frc.team6843.robot.commands.ResetGyro;
import org.usfirst.frc.team6843.robot.commands.ResetRotatedToTarget;
import org.usfirst.frc.team6843.robot.commands.RotateTo;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	/** Used to enforce drive stick dead zones. */
	public static final double DEAD_ZONE = 0.1;
	public static final int DRIVE_AXIS = 1;
	public static final int CURVE_AXIS = 4;

	/** Use these to turn the front throttles into buttons. */
	private static final int LEFT_FRONT_THROTTLE = 2;
	private static final int RIGHT_FRONT_THROTTLE = 3;
	private static final double THROTTLE_AS_BUTTON_THRESHOLD = 0.5;

	private final XboxController driver = new XboxController(0);
	private final Button driverY = new JoystickButton(driver, 4);
	private final Button driverB = new JoystickButton(driver, 2);
	private final Button driverA = new JoystickButton(driver, 1);
	private final Button driverX = new JoystickButton(driver, 3);
	private final Button driverBumperLeft = new JoystickButton(driver, 5);
	private final Button driverBumperRight = new JoystickButton(driver, 6);
	private final Button driverBack = new JoystickButton(driver, 7);
	private final Button driverStart = new JoystickButton(driver, 8);
	private final Trigger driverLeftThrottleButton = new ThrottleButton(driver, LEFT_FRONT_THROTTLE);
	private final Trigger driverRightThrottleButton = new ThrottleButton(driver, RIGHT_FRONT_THROTTLE);
	private final Button driverPOV90 = new POVButton(driver, 90);

	public OI() {
		driverY.whenPressed(new RotateTo(0.0));
		driverB.whenPressed(new RotateTo(90.0));
		driverA.whenPressed(new RotateTo(180.0));
		driverX.whenPressed(new RotateTo(-90.0));
		driverBumperLeft.whenPressed(new RotateTo(-28.75)); // Near left rocket
		driverLeftThrottleButton.whenActive(new RotateTo(-151.25)); // Far left rocket
		driverBumperRight.whenPressed(new RotateTo(28.75)); // Near right rocket
		driverRightThrottleButton.whenActive(new RotateTo(151.25)); // Far right rocket

		// driverBack.whenPressed(new DriveTo(-100));
		// driverStart.whenPressed(new DriveTo(100));
		// driverBack.whileHeld(new DriveTillCancelled(false));
		driverBack.whenPressed(new KillAll());
		// driverStart.whileHeld(new DriveTillCancelled(true));
		driverStart.whileHeld(new ApproachTarget());
		driverStart.whenReleased(new ResetRotatedToTarget());
		driverPOV90.whenPressed(new ResetGyro()); // TODO change to two button trigger
	}

	/**
	 * @return the drive power using cubed inputs.
	 */
	public double getDrivePower() {
		double drivePower = -driver.getRawAxis(DRIVE_AXIS);
		if (Math.abs(drivePower) < DEAD_ZONE) {
			drivePower = 0.0;
		}
		return Math.pow(drivePower, 3.0);
	}

	/**
	 * @return the drive power using inputs to the fifth.
	 */
	public double getCurvePower() {
		double curvePower = driver.getRawAxis(CURVE_AXIS);
		if (Math.abs(curvePower) < DEAD_ZONE) {
			curvePower = 0.0;
		}
		return Math.pow(curvePower, 5.0);
	}

	/**
	 * A custom trigger to turn a throttle into a button.
	 */
	private static class ThrottleButton extends Trigger {
		private final XboxController stick;
		private final int axis;

		public ThrottleButton(final XboxController stick, int axis) {
			this.stick = stick;
			this.axis = axis;
		}

		public boolean get() {
			return Math.abs(this.stick.getRawAxis(this.axis)) > THROTTLE_AS_BUTTON_THRESHOLD;
		}
	}
}
