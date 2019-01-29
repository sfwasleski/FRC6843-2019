/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6843.robot;

import org.usfirst.frc.team6843.robot.commands.ClearEncoders;
import org.usfirst.frc.team6843.robot.commands.DistDrive;
import org.usfirst.frc.team6843.robot.commands.DistDriveRev;
import org.usfirst.frc.team6843.robot.commands.ExampleCommand;
import org.usfirst.frc.team6843.robot.commands.RightTurn;
import org.usfirst.frc.team6843.robot.commands.RightTurnn;
import org.usfirst.frc.team6843.robot.commands.RotateRight;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	Joystick gamepad = new Joystick(0);
	Button buttonA = new JoystickButton(gamepad, 1);
	Button buttonB = new JoystickButton(gamepad, 2);
	Button buttonX = new JoystickButton(gamepad, 3);
	Button buttonY = new JoystickButton(gamepad, 4);
	Button buttonLB = new JoystickButton(gamepad, 5);
	Button buttonRB = new JoystickButton(gamepad, 6);
	Button buttonBack = new JoystickButton(gamepad, 7);
	Button buttonStart = new JoystickButton(gamepad, 8);
	Button buttonLJoyClick = new JoystickButton(gamepad, 9);
	Button buttonRJoyClick = new JoystickButton(gamepad, 10);
	Button button11 = new JoystickButton(gamepad, 11);
	Button button12 = new JoystickButton(gamepad, 12);
	
	public OI() {
		//buttonLB.whileHeld(new ExampleCommand());
		buttonA.whileHeld(new DistDrive());
		buttonB.whileHeld(new DistDriveRev());
		buttonY.whenPressed( new RotateRight());
		//buttonA.whenPressed(new ClearEncoders());
		//buttonY.whileHeld(new RightTurn());
		//buttonB.whenPressed(new RightTurnn());
	}
	
	public double getVertAxis() {
		return gamepad.getRawAxis(1);
	}
	
	public double getHorizAxis() {
		return gamepad.getRawAxis(4);
	}

	public double getTarget() {
		return getVertAxis() * 1000;
	}
	}
