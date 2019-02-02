/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6843.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team6843.robot.RobotMap;
import org.usfirst.frc.team6843.robot.commands.JoystickTankDrive;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The drive subsystem uses the Talon on board PID to control the speed of all
 * driving. It uses on RIO PID controllers to track headings and distances.
 */
public class DriveSubsystem extends Subsystem {
	/** Suggested rotate in place velocity base for PID output math. */
	public static final double ROTATE_VELOCITY_BASE = 1000.0;

	/** Holds the last PID calculated turn rate. */
	private double gyroTurnRate = 0.0;
	/** Our NavX MXP gyro used as PID input for turns. */
	private final AHRS gyro = new AHRS(SPI.Port.kMXP);
	/** The output target for the turn PID Controller. */
	private final PIDOutput turnPidOutput = new PIDOutput() {
		@Override
		public void pidWrite(double output) {
			gyroTurnRate = output;
		}
	};
	/** The turn PID controller using the above gyro and PID output. */
	private final PIDController turnController = new PIDController(0.2, 0.0, 0.0, 0.0, gyro, turnPidOutput);

	/** The target distance in encoder clicks. */
	private int distTarget = 0;
	/** Holds the last PID calculated distance drive velocity. */
	private double distDriveRate = 0.0;
	private PIDSource distDriveSource = new DistDrivePIDSource();
	private PIDOutput distDriveOutput = new PIDOutput() {
		@Override
		public void pidWrite(double output) {
			distDriveRate = output;
		}
	};
	private final PIDController distController = new PIDController(0.01, 0.0, 0.0, 0.0, distDriveSource,
			distDriveOutput);

	private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_1);
	// private final WPI_TalonSRX leftMotor2 = new
	// WPI_TalonSRX(RobotMap.LEFT_MOTOR_2);
	private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_1);
	// private final WPI_TalonSRX rightMotor2 = new
	// WPI_TalonSRX(RobotMap.RIGHT_MOTOR_2);
	private final DifferentialDrive drive = new DifferentialDrive(leftMotor1, rightMotor1);

	// leftEncoderVelocity 1080
	// rightEncoderVelocity 1080

	/**
	 * Creates the components of the subsystem and initialized them all.
	 */
	public DriveSubsystem() {
		rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
		rightMotor1.setSensorPhase(true);
		// set the peak, nominal outputs, and deadband
		rightMotor1.configNominalOutputForward(0, 100);
		rightMotor1.configNominalOutputReverse(0, 100);
		rightMotor1.configPeakOutputForward(1, 100);
		rightMotor1.configPeakOutputReverse(-1, 100);
		// set closed loop gains in slot0
		rightMotor1.config_kF(0, .25, 100); // current was .265
		rightMotor1.config_kP(0, 0.1, 100); // P Value: 0.1
		rightMotor1.config_kI(0, 0, 100);
		rightMotor1.config_kD(0, 0, 100);

		leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
		leftMotor1.setSensorPhase(true);
		// set the peak, nominal outputs, and deadband
		leftMotor1.configNominalOutputForward(0, 100);
		leftMotor1.configNominalOutputReverse(0, 100);
		leftMotor1.configPeakOutputForward(1, 100);
		leftMotor1.configPeakOutputReverse(-1, 100);
		// set closed loop gains in slot0
		leftMotor1.config_kF(0, 0.2578, 100);
		leftMotor1.config_kP(0, 0.1, 100); // P value1: 0.117 value2: .105
		leftMotor1.config_kI(0, 0, 100);
		leftMotor1.config_kD(0, 0, 100);

		leftMotor1.setNeutralMode(NeutralMode.Brake);
		rightMotor1.setNeutralMode(NeutralMode.Brake);
		// leftMotor1.set(ControlMode.PercentOutput, 0.0);
		// leftMotor2.set(ControlMode.Follower, RobotMap.LEFT_MOTOR_1);
		// rightMotor1.set(ControlMode.PercentOutput, 0.0);
		// rightMotor2.set(ControlMode.Follower, RobotMap.RIGHT_MOTOR_1);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(2.0f);
		turnController.setContinuous(true);
		turnController.disable();
	}

	public double getGyroTurnRate() {
		return gyroTurnRate;
	}

	public double getGyroAngle() {
		return gyro.getYaw();
	}

	public boolean isTurnOnTarget() {
		return turnController.onTarget();
	}

	public void startTurn(double targetAngle) {
		turnController.enable();
		turnController.setSetpoint(targetAngle);

	}

	public void endTurn() {
		turnController.reset();
		this.gyroTurnRate = 0.0;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Gyro", gyro.getYaw());
	}

	public double getLeftPosition() {
		double leftRawPos = leftMotor1.getSelectedSensorPosition(0);
		double leftUnitPos = leftRawPos / 1440;
		double leftInchPos = leftUnitPos * 18.85;
		return leftInchPos;
	}

	public void clearEncoders() {
		rightMotor1.setSelectedSensorPosition(0, 0, 100);
		leftMotor1.setSelectedSensorPosition(0, 0, 100);

	}

	public double getRightPosition() {
		double rightRawPos = -rightMotor1.getSelectedSensorPosition(0);
		double rightUnitPos = rightRawPos / 1440;
		double rightInchPos = rightUnitPos * 18.85;
		return rightInchPos;
	}

	public double getLeftVelocity() {
		return leftMotor1.getSelectedSensorVelocity(0);
	}

	public double getRightVelocity() {
		return rightMotor1.getSelectedSensorVelocity(0);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new JoystickTankDrive());
	}

	public void velocityDrive(double leftPower, double rightPower) {
		leftMotor1.set(ControlMode.Velocity, leftPower);
		rightMotor1.set(ControlMode.Velocity, rightPower);
	}

	public void arcadeDrive(double power, double curve) {
		drive.arcadeDrive((-1 * power), (1 * curve));
	}

	public void stop() {
		leftMotor1.set(ControlMode.Velocity, 0);
		rightMotor1.set(ControlMode.Velocity, 0);

	}

	/**
	 * A distance drive PID source that returns raw clicks remaining.
	 */
	private class DistDrivePIDSource implements PIDSource {
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
		}

		@Override
		public double pidGet() {
			int leftRawPos = leftMotor1.getSelectedSensorPosition(0);
			int rightRawPos = -rightMotor1.getSelectedSensorPosition(0);
			int aveRawPos = (leftRawPos + rightRawPos) / 2;
			return distTarget - aveRawPos;
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
	}
}
