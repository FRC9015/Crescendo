package frc.robot.Commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.InputManager;
import static frc.robot.RobotContainer.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.SwerveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import static java.lang.Math.*;


public class NoteDrive extends Command {
    PIDController notePID = new PIDController(0.1, 0.0, 0.0);
    SlewRateLimiter xVelocityFilter = new SlewRateLimiter(SwerveConstants.slewRateLimit);
    SlewRateLimiter yVelocityFilter = new SlewRateLimiter(SwerveConstants.slewRateLimit);

    public NoteDrive() {
        addRequirements(SWERVE);
        notePID.enableContinuousInput(-180, 180);
    }
    @Override
    public void initialize() {
        notePID.reset();
    }
    @Override
    public void execute() {
        
        
        if(LIMELIGHT_INTERFACE.notePose().isEmpty()){
            return;
        }
        
        double targetRotation = LIMELIGHT_INTERFACE.getNoteAngle().minus(SWERVE.getPose().getTranslation()).getAngle().getDegrees();

        double[] inputXYZ = InputManager.getInstance().getDriverXYZAxes();
        double inputX = inputXYZ[0];
        double inputY = inputXYZ[1];
        double inputMagnitude = Math.hypot(inputX, inputY);
        inputMagnitude = MathUtil.applyDeadband(inputMagnitude, 0.1);
        double inputDir = Math.atan2(inputY, inputX);
        double forwardDirectionSign = (RobotContainer.IsRed() ? -1.0 : 1.0);

        double xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * SwerveConstants.maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);

        double yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * SwerveConstants.maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);

        notePID.setSetpoint(targetRotation);
        double rotationalVelocity = notePID.calculate(SWERVE.getPose().getTranslation().getAngle().getDegrees(), targetRotation);//LIMELIGHT_INTERFACE.getNoteAngle().getAngle().getDegrees());
        SWERVE.drive(-yVelocity, -xVelocity, rotationalVelocity);
        
        
       SmartDashboard.putNumber("RotationPID", notePID.getPositionError());

        RobotContainer.logPID("notePID", notePID);
    }

   @Override
   public boolean isFinished() {
       return Math.abs(notePID.getPositionError()) < 3 || LIMELIGHT_INTERFACE.notePose().isEmpty();
   }
   @Override
   public void end(boolean interrupted) {
       SWERVE.drive(0, 0, 0);
   }

}
