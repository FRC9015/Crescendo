package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.subsystems.Swerve.Drive;

import java.util.function.DoubleSupplier;

public class JoystickDrive extends Command {

    private SlewRateLimiter xVelocityFilter = new SlewRateLimiter(Constants.SwerveConstants.slewRateLimit);
    private SlewRateLimiter yVelocityFilter = new SlewRateLimiter(Constants.SwerveConstants.slewRateLimit);

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;
    private Drive drive;

    public JoystickDrive(Drive drive,
                         DoubleSupplier xSupplier,
                         DoubleSupplier ySupplier,
                         DoubleSupplier omegaSupplier){
        addRequirements(drive);
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;

        this.drive = drive;
    }

    @Override
    public void execute() {
        // Apply deadband
        double linearMagnitude =
                MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), 0.2);
        Rotation2d linearDirection =
                new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), 0.2);

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = Math.copySign(omega * omega, omega);

        // Calcaulate new linear velocity
        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

        double xVelocity = xVelocityFilter.calculate(linearVelocity.getX() * Constants.SwerveConstants.maxSpeed);
        double yVelocity = yVelocityFilter.calculate(linearVelocity.getY() * Constants.SwerveConstants.maxSpeed);

        // Convert to field relative speeds & send command
        boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xVelocity,
                        yVelocity,
                        omega * Constants.SwerveConstants.angularSpeed,
                        isFlipped
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation()));
    }
}
