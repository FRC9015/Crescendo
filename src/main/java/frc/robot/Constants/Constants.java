// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	
	public static class OperatorConstants {
		public static final int driverControllerPort = 0;
		public static final int operatorControllerPort = 1;
	}

	public static class InputConstants{
		public static String triggerPressThresholdKey = "PressThreshold";
		public static double defaultTriggerPressThreshold = 0.15;
	}

	public static class SwerveConstants {
		public static final double maxSpeed = Units.feetToMeters(10);
	}

	public static class ShooterConstants {
		public static final int speakerShooterMotorTopID = 51;
		public static final int speakerShooterMotor2ID = 52;

		public static final int ampShooterMotor1ID = 61;
		public static final int ampShooterMotor2ID = 62;

		public static final int pivotMotor1ID = 71;
	}

	public static class IntakeConstants {
		public static final int intakeMotor1ID = 41;
		public static final int intakeMotor2ID = 42;

	}

	public static class LEDConstants {

	}
	public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
			new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
			new PIDConstants(3, 0.0, 0.0), // Rotation PID constants
			SwerveConstants.maxSpeed, // Max module speed, in m/s
			Units.feetToMeters(1), // Drive base radius in meters. Distance from robot center to furthest module.
			new ReplanningConfig() // Default path replanning config. See the API for the options here
			);

	
	public static class PigeonConstants {
		public static final int pigeonID = 30; //TODO Remove this Later for YAGSL Json Instance
	}
}
