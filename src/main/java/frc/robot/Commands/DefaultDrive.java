// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.Constants.Constants.SwerveConstants.slewRateLimit;
import static frc.robot.Constants.Constants.SwerveConstants.maxSpeed;
import static frc.robot.Constants.Constants.SwerveConstants.angularSpeed;
import static frc.robot.RobotContainer.SWERVE;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.cos;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.InputManager;
import frc.robot.RobotContainer;


/** An example command that uses an example subsystem. */
public class DefaultDrive extends Command {
	

	SlewRateLimiter xVelocityFilter = new SlewRateLimiter(slewRateLimit);
	SlewRateLimiter yVelocityFilter = new SlewRateLimiter(slewRateLimit);

//	PIDController headingPID = new PIDController(1.4,0.1,0);
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public DefaultDrive() {
		addRequirements(SWERVE);
	}



	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double[] inputXYZ = InputManager.getInstance().getDriverXYZAxes();
		double inputX = inputXYZ[0];
		double inputY = inputXYZ[1];
		double inputZ = inputXYZ[2];
		inputZ = MathUtil.applyDeadband(inputZ, 0.1);
		double inputMagnitude = Math.hypot(inputX, inputY);
		inputMagnitude = MathUtil.applyDeadband(inputMagnitude, 0.1);
		double inputDir = Math.atan2(inputY, inputX);
		double forwardDirectionSign = (RobotContainer.IsRed() ? -1.0 : 1.0);

		double xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);

		double yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);

		double rotationalVelocity = (inputZ * angularSpeed );

		

		
		
		SWERVE.drive(-yVelocity, -xVelocity, -rotationalVelocity);
		
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

	
}
