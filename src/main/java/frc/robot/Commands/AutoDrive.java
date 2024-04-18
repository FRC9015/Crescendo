package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;
import static frc.robot.RobotContainer.SWERVE;
import static java.lang.Math.abs;

public class AutoDrive extends Command {
    PIDController limelightPID = new PIDController(6, 1, 0.025);

    public AutoDrive() {
        addRequirements(SWERVE);
        limelightPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        limelightPID.reset();
    }

    @Override
    public void execute() {
        double rotationalVelocity = limelightPID.calculate(SWERVE.getPose().getRotation().getRadians(), LIMELIGHT_INTERFACE.getSpeakerAngle().getRadians());
        SWERVE.drive(0, 0, rotationalVelocity);
        Logger.recordOutput("Drive error", limelightPID.getPositionError());

    }

    @Override
    public boolean isFinished() {
        return (abs(Units.radiansToDegrees(limelightPID.getPositionError())) < 3);
    }

    @Override
    public void end(boolean interrupted) {
        SWERVE.drive(0,0,0);
    }
}

