// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends Command {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public DefaultDrive() {
		addRequirements(swerve);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double vx = driveController.getLeftX();
		double vy = -driveController.getLeftY();
		double w = -driveController.getRightX();
		double mag = Math.hypot(vx, vy);
		double ma2 = MathUtil.applyDeadband(mag, 0.15);
		double theta = Math.atan2(vy, vx);
		vx = cos(theta) * ma2 * maxSpeed;
		vy = sin(theta) * ma2 * maxSpeed;

		ChassisSpeeds speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, MathUtil.applyDeadband(w, 0.15), imu.yaw());
		swerve.drive(speeds);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
