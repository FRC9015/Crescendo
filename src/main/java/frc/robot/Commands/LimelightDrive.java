package frc.robot.Commands;

import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;
import static frc.robot.RobotContainer.SWERVE;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.InputManager;
import frc.robot.RobotContainer;

import static frc.robot.Constants.Constants.SwerveConstants;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.littletonrobotics.junction.Logger;

public class LimelightDrive extends Command{
    public LimelightDrive(){
        addRequirements(SWERVE);
    }

    PIDController limelightPID = new PIDController(0.2, 0.01, 0.0025);

    SlewRateLimiter xVelocityFilter = new SlewRateLimiter(SwerveConstants.slewRateLimit);
    SlewRateLimiter yVelocityFilter = new SlewRateLimiter(SwerveConstants.slewRateLimit);
    @Override
    public void initialize() {
        limelightPID.reset();
    }

    @Override
    public void execute() {
        double[] inputXYZ = InputManager.getInstance().getDriverXYZAxes();
		double inputX = inputXYZ[0];
		double inputY = inputXYZ[1];
		double inputMagnitude = Math.hypot(inputX, inputY);
		inputMagnitude = MathUtil.applyDeadband(inputMagnitude, 0.1);
		double inputDir = Math.atan2(inputY, inputX);
		double forwardDirectionSign = (RobotContainer.IsRed() ? -1.0 : 1.0);

		double xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * SwerveConstants.maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);

		double yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * SwerveConstants.maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);

		double rotationalVelocity = limelightPID.calculate(SWERVE.getPose().getRotation().getDegrees(), LIMELIGHT_INTERFACE.getSpeakerAngle().getDegrees());
		Logger.recordOutput("AutoAim/PID/Error", limelightPID.getPositionError());
        SWERVE.drive(-yVelocity, -xVelocity, rotationalVelocity);
        SmartDashboard.putNumber("Drive error", limelightPID.getPositionError());

        if(abs(limelightPID.getPositionError()) < 1){
            LIMELIGHT_INTERFACE.Error = true;
        }else{
            LIMELIGHT_INTERFACE.Error = false;
        }
    }
}
