package frc.robot.subsystems;

import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class PoseEstimator extends SubsystemBase{

    private GenericEntry currentPos = Shuffleboard.getTab("swerve").add("curr_pos",new double[3]).getEntry();
    
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private final SwerveSubsystem swerveSubsystem;
    private final Pigeon pigeon;
    private Field2d field = new Field2d();

    private Pose2d originPose;

    public PoseEstimator(SwerveSubsystem swerveSubsystem, Pigeon pigeon, Pose2d initialPose){
        this.swerveSubsystem = swerveSubsystem;
        this.pigeon = pigeon;
        originPose = initialPose;

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerveSubsystem.getKinematics(),
                pigeon.getYawAsRotation2d(),
                swerveSubsystem.getPositions(),
                initialPose,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.25, 0.25, 0.25));

        initShuffleboard();
    }

    private void initShuffleboard(){
        Shuffleboard.getTab("swerve").add(field);
    }

    public void updatePoseEstimator() {
        if (LIMELIGHT_INTERFACE.tagCheck()) {

            double getLatestResultTime = System.currentTimeMillis();
            LimelightHelpers.Results result = LimelightHelpers.getLatestResults("limelight").targetingResults;
            Logger.recordOutput("PoseEstimator/Profiler/LimelightLatestResult", getLatestResultTime-System.currentTimeMillis());
            if (LimelightHelpers.getTV("limelight")) {

                double tl = result.latency_pipeline;
                double cl = result.latency_capture;

                double timeStamp = Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);
                
                double getLatestResultTime2 = System.currentTimeMillis();
                swerveDrivePoseEstimator.addVisionMeasurement(result.getBotPose2d_wpiBlue(), timeStamp);
                Logger.recordOutput("PoseEstimator/Profiler/addVisionMeasurement", getLatestResultTime2-System.currentTimeMillis());

                Logger.recordOutput("limelight/botPose", result.getBotPose2d_wpiBlue());
                

            }
        }
        field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
        


    }

    @Override
    public void periodic() {
        swerveDrivePoseEstimator.update(pigeon.getYawAsRotation2d(), swerveSubsystem.getPositions());

        Logger.recordOutput("Odom/Pose", getEstimatedPose());

        updatePoseEstimator();
    }

    public Pose2d getEstimatedPose(){
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(){
        pigeon.resetYaw(originPose.getRotation().getDegrees());

        swerveDrivePoseEstimator.resetPosition(originPose.getRotation(), swerveSubsystem.getPositions(), originPose);
    }

    public void resetOdomGivenPose2d(Pose2d pose) {
		swerveDrivePoseEstimator.resetPosition(pigeon.getYawAsRotation2d(), swerveSubsystem.getPositions(), pose);
	  }

}
