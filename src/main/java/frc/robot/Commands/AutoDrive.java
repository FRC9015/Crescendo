package frc.robot.Commands;

import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;
import static frc.robot.RobotContainer.SWERVE;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.InputManager;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.SwerveConstants;

public class AutoDrive extends Command{
     public AutoDrive(){
        addRequirements(SWERVE);
    }

    PIDController limelightPID = new PIDController(0.2, 0.01, 0.0025);

    @Override
    public void initialize() {
        limelightPID.reset();
    }

    @Override
    public void execute() {
		double rotationalVelocity = limelightPID.calculate(SWERVE.getPose().getRotation().getDegrees(), LIMELIGHT_INTERFACE.getSpeakerAngle().getDegrees());
        SWERVE.drive(0, 0, rotationalVelocity);
        SmartDashboard.putNumber("Drive error", limelightPID.getPositionError());
       
    }

    @Override
    public boolean isFinished() {
         return (abs(limelightPID.getPositionError()) < 1);
    }

   
}

