/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6843.robot;

import java.util.logging.Logger;

import org.usfirst.frc.team6843.robot.commands.DriveTo;
import org.usfirst.frc.team6843.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team6843.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	private static Robot INSTANCE;

	private VisionSubsystem visionSubsystem;
	private DriveSubsystem driveSubsystem;
	private OI oi;
	private Logger logger;
	private SendableChooser<StartOrientation> startOrientationChooser;
	private double startHeading = 0.0;
	private double lastCompletedRotateTo = 0.0;
	private SendableChooser<Command> auto_chooser;
	private Command autonomousCommand;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		/*
		 * m_oi = new OI(); m_chooser.addDefault("Default Auto", new ExampleCommand());
		 * // chooser.addObject("My Auto", new MyAutoCommand());
		 * SmartDashboard.putData("Auto mode", m_chooser);
		 */

		INSTANCE = this;
		this.logger = Logger.getLogger(this.getClass().getName());
		this.visionSubsystem = new VisionSubsystem();
		this.driveSubsystem = new DriveSubsystem();
		this.oi = new OI();
		this.auto_chooser = new SendableChooser<>();
		this.auto_chooser.setDefaultOption("Drive 3 pts", new DriveTo(50.0));
		this.auto_chooser.addOption("Drive too far", new DriveTo(100.0));
		SmartDashboard.putData("Auto Select", this.auto_chooser);
		// chooser.addObject("My Auto", new MyAutoCommand());
		// SmartDashboard.putData("Auto mode", auto_chooser);
		this.startOrientationChooser = new SendableChooser<>();
		this.startOrientationChooser.setDefaultOption("Backward", StartOrientation.BACKWARD);
		this.startOrientationChooser.addOption("Forward", StartOrientation.FORWARD);
		this.startOrientationChooser.addOption("Left", StartOrientation.LEFT);
		this.startOrientationChooser.addOption("Right", StartOrientation.RIGHT);
		SmartDashboard.putData("Start orientation", this.startOrientationChooser);
	}

	public static Robot getInstance() {
		if (INSTANCE == null) {
			throw new IllegalStateException("Robot.getInstance() was called before Robot.robotInit() was called.");
		}
		return INSTANCE;
	}

	public Logger getLogger() {
		if (this.logger == null) {
			throw new IllegalStateException("Robot.getLogger() was called before Robot.robotInit() was called.");
		}
		return this.logger;
	}

	/**
	 * @return the last reported completed rotate to heading.
	 */
	public double getLastCompletedRotateTo() {
		return this.lastCompletedRotateTo;
	}

	/**
	 * @param lastCompletedRotateTo the latest completed rotate to target.
	 */
	public void setLastCompletedRotateTo(double lastCompletedRotateTo) {
		this.lastCompletedRotateTo = lastCompletedRotateTo;
	}

	public OI getOI() {
		if (this.oi == null) {
			throw new IllegalStateException("Robot.getOI() was called before Robot.robotInit() was called.");
		}
		return this.oi;
	}

	public DriveSubsystem getDriveSubsystem() {
		if (this.driveSubsystem == null) {
			throw new IllegalStateException(
					"Robot.getDriveSubsystem() was called before Robot.robotInit() was called.");
		}
		return this.driveSubsystem;
	}

	public VisionSubsystem getVisionSubsystem() {
		if (this.visionSubsystem == null) {
			throw new IllegalStateException(
					"Robot.getVisionSubsystem() was called before Robot.robotInit() was called.");
		}
		return this.visionSubsystem;
	}

	/**
	 * Note that this is not accurate until after {@link #autonomousInit()}.
	 * 
	 * @return the starting heading according to the auto selections.
	 */
	public double getStartHeading() {
		return startHeading;
	}

	@Override
	public void robotPeriodic() {
		getDriveSubsystem().updateDashboard();
		getVisionSubsystem().updateDashboard();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		updateStartingHeading();

		autonomousCommand = auto_chooser.getSelected();
		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	private void updateStartingHeading() {
		StartOrientation orientation = startOrientationChooser.getSelected();
		switch (orientation) {
		case FORWARD:
			this.startHeading = 0.0;
			break;
		case BACKWARD:
			this.startHeading = 180.0;
			break;
		case LEFT:
			this.startHeading = -90.0;
			break;
		case RIGHT:
			this.startHeading = 90.0;
			break;
		default:
			this.startHeading = 180.0;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		updateStartingHeading();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	public static enum StartOrientation {
		FORWARD, BACKWARD, LEFT, RIGHT;
	}
}
