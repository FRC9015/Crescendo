package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.util.Map;

public class PoseEstimator extends SubsystemBase{

    private GenericEntry currentPos = Shuffleboard.getTab("swerve").add("curr_pos",new double[3]).getEntry();
    private SimpleWidget visionMeasurementToggle =
            Shuffleboard.getTab("swerve")
                    .add("Add Vision Measurement", false)
                    .withWidget(BuiltInWidgets.kToggleButton);
    private boolean addVisionMeasurement;
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
                VecBuilder.fill(0.1, 0.1, 0.1));

        initShuffleboard();
    }

    private void initShuffleboard(){
        Shuffleboard.getTab("swerve").add(field);
    }

    @Override
    public void periodic() {
        swerveDrivePoseEstimator.update(pigeon.getYawAsRotation2d(), swerveSubsystem.getPositions());

        if (addVisionMeasurement) {
            LimelightHelpers.Results result = LimelightHelpers.getLatestResults("limelight").targetingResults;
            if (LimelightHelpers.getTV("limelight")) {

                //System.out.println(result.getBotPose2d());
                double tl = result.latency_pipeline;
                double cl = result.latency_capture;

                double timeStamp = Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);


                swerveDrivePoseEstimator.addVisionMeasurement(result.getBotPose2d(), timeStamp);

            }
        }
        field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());

        double[] curr_pos = {
                swerveDrivePoseEstimator.getEstimatedPosition().getX(),
                swerveDrivePoseEstimator.getEstimatedPosition().getY(),
                swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians()
        };
        currentPos.setDoubleArray(curr_pos);

        addVisionMeasurement = visionMeasurementToggle.getEntry().get().getBoolean();
    }

    public Pose2d getEstimatedPose(){
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(){
        pigeon.resetYaw(originPose.getRotation().getDegrees());

        swerveDrivePoseEstimator.resetPosition(originPose.getRotation(), swerveSubsystem.getPositions(), originPose);
    }
}
