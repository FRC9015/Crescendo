// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends Command {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public DefaultDrive() {
		addRequirements(swerve);
	}

	private SimpleWidget speedMultiplierWidget = Shuffleboard.getTab("Drive")
		.add("Max Speed", 0.5)
		.withWidget(BuiltInWidgets.kNumberSlider)
		.withProperties(Map.of("min", 0, "max", 1)); // specify widget properties here
		
		
		private SimpleWidget angularMultiplierWidget = Shuffleboard.getTab("Drive")
		.add("Max Angular Speed", 0.5)
		.withWidget(BuiltInWidgets.kNumberSlider)
		.withProperties(Map.of("min", 0, "max", 1)); // specify widget properties here

		private double speedMultiplier; 
		private double angularMultiplier; 

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
		double sign = (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red) ? 1.0 : -1.0);

		
		xVelocity = cos(velocityDir) * deadbandSpeed * maxSpeed * speedMultiplier * sign;
		yVelocity = sin(velocityDir) * deadbandSpeed * maxSpeed * speedMultiplier * sign;
		rotationalVelocity = rotationalVelocity * angularSpeed * angularMultiplier;
		
		ChassisSpeeds speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, swerve.pose_est.getEstimatedPosition().getRotation());
		swerve.drive(speeds);
	}
	public void periodic() {
		speedMultiplier = speedMultiplierWidget.getEntry().get().getDouble()/100;
		angularMultiplier = angularMultiplierWidget.getEntry().get().getDouble()/100;
	}
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
