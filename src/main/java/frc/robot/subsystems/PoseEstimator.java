package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase{
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private SwerveSubsystem swerveSubsystem;
    private Pigeon pigeon;


    public PoseEstimator(SwerveSubsystem swerveSubsystem, Pigeon pigeon){
        this.swerveSubsystem = swerveSubsystem;
        this.pigeon = pigeon;

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerveSubsystem.getKinematics(),
                pigeon.getYawAsRotation2d(),
                swerveSubsystem.getPositions(),
                new Pose2d());
    }
}
