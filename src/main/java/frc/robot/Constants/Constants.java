// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.Constants.SwervePIDControllerConstants.*;


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
		public static final int DRIVER_CONTROLLER_PORT = 0;
	}

	public static class SwerveConstants {
		public static final double maxSpeed = Units.feetToMeters(19.3);
		public static final double angularSpeed = maxSpeed / (Math.hypot(robotLength, robotWidth) / 2);
		public static final double driveSlewRateLimit = 15;
		public static final double turningSlewRateLimit = 20;
		public static final double dtSeconds = 0.02;
		public static final double driveBaseRadius = Units.inchesToMeters(12);
	}

	public static class PigeonConstants {
		public static final int pigeonID = 30;
	}

	public static class IntakeConstants {
		public static final int intakeMotor1ID = 40;
		public static final int intakeMotor2ID = 41;
		public static final int intakeMotor3ID = 42;
	}

	public static class ShooterConstants {
		public static final int pivotMotor = 50;
		public static final int speakerShooterMotor1ID = 51;
		public static final int speakerShooterMotor2ID = 52;
		public static final int ampShooterMotor1ID = 53;
		public static final int ampShooterMotor2ID = 54;
	}
	public static class LEDConstants {

	}
	public static final double robotWidth = Units.inchesToMeters(26); // TODO: This must be tuned to specific robot
	public static final double robotLength = Units.inchesToMeters(26); // TODO: This must be tuned to specific robot
	public static final double wheelRatio = Units.inchesToMeters(2);
	public static final double gearRatio = 6.12;
	public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
			new PIDConstants(driveP, driveI, driveD), // Translation PID constants
			new PIDConstants(turnP, turnI, turnD), // Rotation PID constants
			SwerveConstants.maxSpeed, // Max module speed, in m/s
			Units.inchesToMeters(12), //TODO Change this to competition robot radius Drive base radius in meters. Distance from robot center to furthest module.
			new ReplanningConfig() // Default path re-planning config. See the API for the options here
			);

	public static class SwervePIDControllerConstants {
		public static final double driveP = 1.5;
		public static final double driveI = 0;
		public static final double driveD = 0;

		public static final double turnP = 2.5;
		public static final double turnI = 0;
		public static final double turnD = 0;
	}
}
