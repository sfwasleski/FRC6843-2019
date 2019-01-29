package org.usfirst.frc.team6843.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;

public class Autonomous_Command {
	   String gameData; {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(gameData.charAt(0) == 'R')
		{
			// Go straight 168 in. forward
			// Turn 90 degrees left
			// Raise lift for switch
			// Go forward 41 in. (DriveForward)
			
			// Push cube onto switch
			// Back up 41 in. 
			// Turn 90 degrees right
			// Go forward 100 in.
		} else {
			if(gameData.charAt(1) == 'R')
			{
				// Go straight 323 in. forward
				// Turn 90 degrees left
				// Raise lift for scale
				// Go forward 27 in. (make another constant for other Auto modes)
				// Push cube onto scale
				// Back up 27 in.
				// Turn 90 degrees right
			} else {
				// Go straight forward 200 in.
			}
		}
	   }
}		