package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.Utils.Transform2d;


public class Odometry extends SubsystemBase{
    
	private Pigeon imu;
    private SwerveSubsystem swerve;
	private LimelightInterface limelight;
	public SwerveDrivePoseEstimator pose_est;

	public Odometry(Pigeon imu, SwerveSubsystem swerve, LimelightInterface limlight){
		this.imu = imu;
		this.swerve = swerve;
		this.limelight = limlight;
	}
    
   
    
	Field2d field = new Field2d();
    final ArrayList<Transform2d> odoms = new ArrayList<>();
    private GenericEntry currentPos = Shuffleboard.getTab("swerve").add("curr_pos",new double[3]).getEntry();
    Transform2d origin = new Transform2d();
    
    
    
    
    public Transform2d acc() {
		return odoms.get(odoms.size() - 1);
	}

    public Transform2d now() {
		return origin.mul(acc());
	}
    
    @Override
	public void periodic() {
        
        pose_est.update(imu.getYawAsRotation2d(),swerve.getPositions());
			LimelightHelpers.Results result = LimelightHelpers.getLatestResults("limelight").targetingResults;
			
			if(LimelightHelpers.getTV("limelight")){

				double tl = result.latency_pipeline;
				double cl = result.latency_capture;
				
				double timeStamp = Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0);


				pose_est.addVisionMeasurement(result.getBotPose2d(), timeStamp);

			}
			field.setRobotPose(pose_est.getEstimatedPosition());
			
			double[] curr_pos = {
                pose_est.getEstimatedPosition().getX(),
                pose_est.getEstimatedPosition().getY(),
                pose_est.getEstimatedPosition().getRotation().getRadians()
        };
		currentPos.setDoubleArray(curr_pos);
    }
    	
    public void init(Pose2d init_pose){
		pose_est = new SwerveDrivePoseEstimator(
			swerve.kinematics,
			imu.getYawAsRotation2d(),
			swerve.getPositions(),
			init_pose,
			VecBuilder.fill(0.1, 0.1, 0.1),
			VecBuilder.fill(0.1, 0.1, 0.1));
		
	}
    public void initShuffleboard(){
		Shuffleboard.getTab("swerve").add(field);
	}
}
