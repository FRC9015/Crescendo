// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.InputManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/** An example command that uses an example subsystem. */
public class DefaultDrive extends Command {
	

	SlewRateLimiter xVelocityFilter = new SlewRateLimiter(slewRateLimit);
	SlewRateLimiter yVelocityFilter = new SlewRateLimiter(slewRateLimit);

	PIDController headingPID = new PIDController(1.4,0.1,0);
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public DefaultDrive() {
		addRequirements(SWERVE);
	}

	@Override
	public void initialize() {
		
		headingPID.setSetpoint(POSE_ESTIMATOR.getEstimatedPose().getRotation().getRadians());
		headingPID.enableContinuousInput(-PI, PI);
	
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double[] inputXYZ = InputManager.getInstance().getDriverXYZAxes();
		double inputX = inputXYZ[0];
		double inputY = inputXYZ[1];
		double inputZ = inputXYZ[2];
		inputZ = MathUtil.applyDeadband(inputZ, 0.15);
		double inputMagnitude = Math.hypot(inputX, inputY);
		inputMagnitude = MathUtil.applyDeadband(inputMagnitude, 0.15);
		double inputDir = Math.atan2(inputY, inputX);
		double forwardDirectionSign = (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red) ? -1.0 : 1.0);

		double xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * maxSpeed * forwardDirectionSign);

		double yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * maxSpeed * forwardDirectionSign);

		double rotationalVelocity = (inputZ * angularSpeed );

		if(abs(rotationalVelocity) > 1e-10){
			headingPID.setSetpoint(POSE_ESTIMATOR.getEstimatedPose().getRotation().getRadians());
			headingPID.reset();
		}else if(abs(xVelocity) > 1e-10 || abs(yVelocity) > 1e-10){
			rotationalVelocity = -headingPID.calculate(POSE_ESTIMATOR.getEstimatedPose().getRotation().getRadians());
		}
		
		Logger.recordOutput("Swerve/heading/target", headingPID.getSetpoint());
		Logger.recordOutput("Swerve/heading/error", headingPID.getPositionError());
		Logger.recordOutput("Swerve/Recorded", POSE_ESTIMATOR.getEstimatedPose().getRotation().getRadians());
		SWERVE.drive(-yVelocity, -xVelocity, -rotationalVelocity);
		
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

	
}
