package frc.robot.subsystems;

import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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
                VecBuilder.fill(0.4, 0.4, 0.1));

        initShuffleboard();
    }

    private void initShuffleboard(){
        Shuffleboard.getTab("swerve").add(field);
    }

    public void updatePoseEstimator() {
        if (LIMELIGHT_INTERFACE.tagCheck()) {
            LimelightHelpers.Results result = LimelightHelpers.getLatestResults("limelight").targetingResults;
            if (LimelightHelpers.getTV("limelight")) {

                double tl = result.latency_pipeline;
                double cl = result.latency_capture;

                double timeStamp = Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);
                
                
                swerveDrivePoseEstimator.addVisionMeasurement(result.getBotPose2d_wpiBlue(), timeStamp);

                Logger.recordOutput("limelight/botPose", result.getBotPose2d_wpiBlue());
                
            }
        }
        field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
        

        double[] curr_pos = {
                swerveDrivePoseEstimator.getEstimatedPosition().getX(),
                swerveDrivePoseEstimator.getEstimatedPosition().getY(),
                swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians()
        };
        currentPos.setDoubleArray(curr_pos);
    }

    @Override
    public void periodic() {
        swerveDrivePoseEstimator.update(pigeon.getYawAsRotation2d(), swerveSubsystem.getPositions());


        SmartDashboard.putString("BOtPose",getEstimatedPose().toString());
        Logger.recordOutput("Odom/Pose", getEstimatedPose());
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
