package frc.robot.subsystems;

import static frc.robot.Constants.Constants.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConfiguration;

public class SwerveSubsystem extends SubsystemBase {
	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(robotLength / 2, robotWidth / 2), // NW
			new Translation2d(robotLength / 2, -robotWidth / 2), // NE
			new Translation2d(-robotLength / 2, -robotWidth / 2), // SE
			new Translation2d(-robotLength / 2, robotWidth / 2) // SW
			);

	private SwerveModule[] modules = new SwerveModule[] {
		new SwerveModule(SwerveModuleConfiguration.NW, "NW"),
		new SwerveModule(SwerveModuleConfiguration.NE, "NE"),
		new SwerveModule(SwerveModuleConfiguration.SE, "SE"),
		new SwerveModule(SwerveModuleConfiguration.SW, "SW"),
	};

	/**
	 * Moves the Swerve Modules with a certain x and y velocities and rotation.
	 *
	 * @param xVelocity X Velocity in Meters Per Second
	 * @param yVelocity Y Velocity in Meters Per Second
	 * @param rotationalVelocity Rotational Velocity in Radians Per Second
	 */
	public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {
		ChassisSpeeds chassisSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, PIGEON.getYawAsRotation2d());

		SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
		for (int i = 0; i < modules.length; i++) {
			modules[i].setState(states[i]);
		}
	}

	@Override
	public void periodic() {
		for (SwerveModule module : modules) {
			module.periodic();
		}
	}

	public SwerveDriveKinematics getKinematics() { return kinematics; }

	public SwerveModulePosition[] getPositions(){
		SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

		for (int i = 0; i < modules.length; i++) {
			SwerveModule module = modules[i];

			swerveModulePositions[i] = module.getPosition();
		}

		return swerveModulePositions;
	}

	public void fixOffsets() {
		for (SwerveModule module : modules) module.fixOffset();
	}

	public Command printOffsets() {
		return new InstantCommand(this::fixOffsets, this);
	}
}
