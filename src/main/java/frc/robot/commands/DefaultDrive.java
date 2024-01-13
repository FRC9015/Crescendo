// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public DefaultDrive() {
		addRequirements(swerve);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double xVelocity = driveController.getLeftX();
		double yVelocity = -driveController.getLeftY();
		double rotationalVelocity = -driveController.getRightX();
		rotationalVelocity = MathUtil.applyDeadband(rotationalVelocity, 0.1);
		double speed = Math.hypot(xVelocity, yVelocity);
		double deadbandSpeed = MathUtil.applyDeadband(speed, 0.1);
		double velocityDir = Math.atan2(yVelocity, xVelocity);
		xVelocity = cos(velocityDir) * deadbandSpeed * maxSpeed;
		yVelocity = sin(velocityDir) * deadbandSpeed * maxSpeed;
		rotationalVelocity = rotationalVelocity * maxSpeed;


		swerve.drive(xVelocity, yVelocity, rotationalVelocity);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
