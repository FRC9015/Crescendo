// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.InputManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends Command {
	
	SlewRateLimiter xVelocityFilter = new SlewRateLimiter(slewRateLimit);
	SlewRateLimiter yVelocityFilter = new SlewRateLimiter(slewRateLimit);
	SlewRateLimiter rotationalVelocityFilter = new SlewRateLimiter(slewRateLimit);
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public DefaultDrive() {
		addRequirements(SWERVE);
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
		Translation2d moveVelocity = InputManager.getInstance().getControllerXYAxes();
		double xVelocity = moveVelocity.getX();
		double yVelocity = moveVelocity.getY();
		double rotationalVelocity = InputManager.getInstance().getControllerRotationalAxis();
		rotationalVelocity = MathUtil.applyDeadband(rotationalVelocity, 0.1);
		double speed = Math.hypot(xVelocity, yVelocity);
		double deadbandSpeed = MathUtil.applyDeadband(speed, 0.1);
		double velocityDir = Math.atan2(yVelocity, xVelocity);
		double forwardDirectionSign = (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red) ? -1.0 : 1.0);
		
		xVelocity = xVelocityFilter.calculate(cos(velocityDir) * deadbandSpeed * maxSpeed * SWERVE.getSpeedMultiplier() * -forwardDirectionSign);
		
		yVelocity = yVelocityFilter.calculate(sin(velocityDir) * deadbandSpeed * maxSpeed * SWERVE.getSpeedMultiplier() * forwardDirectionSign);
		
		rotationalVelocity = rotationalVelocityFilter.calculate(rotationalVelocity * angularSpeed * SWERVE.getAngularMultiplier());
		
		SWERVE.drive(xVelocity, yVelocity, rotationalVelocity);
		
		SWERVE.velocityGraphUpdate(xVelocity,yVelocity); //TODO Add all data visualization commands to one subsystem
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
