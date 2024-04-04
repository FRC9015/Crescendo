//package frc.robot.Commands;
//
//import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;
//import static frc.robot.RobotContainer.POSE_ESTIMATOR;
//import static frc.robot.RobotContainer.SWERVE;
//
//
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.InputManager;
//import frc.robot.RobotContainer;
//
//import static frc.robot.Constants.Constants.SwerveConstants;
//import static java.lang.Math.cos;
//import static java.lang.Math.sin;
//
//import org.littletonrobotics.junction.Logger;
//
//public class LimelightDrive extends Command{
//    public LimelightDrive(){
//        addRequirements(SWERVE);
//    }
//
//    PIDController L_PID = new PIDController(0.05, 0.005, 0);
//
//    SlewRateLimiter xVelocityFilter = new SlewRateLimiter(SwerveConstants.slewRateLimit);
//    SlewRateLimiter yVelocityFilter = new SlewRateLimiter(SwerveConstants.slewRateLimit);
//    @Override
//    public void initialize() {
//        L_PID.reset();
//    }
//
//    @Override
//    public void execute() {
//        double[] inputXYZ = InputManager.getInstance().getDriverXYZAxes();
//		double inputX = inputXYZ[0];
//		double inputY = inputXYZ[1];
//		double inputMagnitude = Math.hypot(inputX, inputY);
//		inputMagnitude = MathUtil.applyDeadband(inputMagnitude, 0.2);
//		double inputDir = Math.atan2(inputY, inputX);
//		double forwardDirectionSign = (RobotContainer.IsRed() ? 1.0 : -1.0);
//
//		double xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * SwerveConstants.maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);
//
//		double yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * SwerveConstants.maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);
//
//		double rotationalVelocity = L_PID.calculate(POSE_ESTIMATOR.getEstimatedPose().getRotation().getDegrees(), LIMELIGHT_INTERFACE.getSpeakerAngle().getDegrees());
//		Logger.recordOutput("AutoAim/PID/Error", L_PID.getPositionError());
//        SWERVE.drive(-yVelocity, -xVelocity, rotationalVelocity);
//
//    }
//}
