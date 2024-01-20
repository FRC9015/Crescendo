package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.*;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotSelf.RobotSelves;
import frc.robot.Constants.SwerveModuleConfiguration;

import frc.robot.subsystems.IMU;
import edu.wpi.first.math.geometry.Pose2d;
public class SwerveSubsystem extends SubsystemBase {
	
	
	private GenericEntry currentPos = Shuffleboard.getTab("swerve").add("curr_pos",new double[3]).getEntry();
	
	
	private IMU imu = new IMU();

	public SwerveDrivePoseEstimator pose_est;

	Field2d field = new Field2d();
	
	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(robotLength / 2, robotWidth / 2), // NW
			new Translation2d(robotLength / 2, -robotWidth / 2), // NE
			new Translation2d(-robotLength / 2, -robotWidth / 2), // SE
			new Translation2d(-robotLength / 2, robotWidth / 2) // SW
			);

	private SwerveModule[] modules = new SwerveModule[] {
		new SwerveModule(SwerveModuleConfiguration.NW, "NW"),
		new SwerveModule(SwerveModuleConfiguration.NE, "NE"),
		new SwerveModule(SwerveModuleConfiguration.SE, "SE"),
		new SwerveModule(SwerveModuleConfiguration.SW, "SW"),
	};

	public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] pos = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) pos[i] = modules[i].getPosition();
        return pos;
    }
	public void drive(ChassisSpeeds speeds) {
		
		
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
		for (int i = 0; i < modules.length; i++) {
			modules[i].setState(states[i]);
		}
	}


	@Override
	public void periodic() {
		//if statment is so that the telop wont run if selfdrive is on.
		if(!RobotSelves.getAmpSelf() && !RobotSelves.getSpeakerSelf()){
			for (SwerveModule module : modules) {
				module.teleop();
			}
		}
		
			pose_est.update(imu.yaw(),getPositions());
			LimelightHelpers.Results result = LimelightHelpers.getLatestResults("limelight").targetingResults;
			
			if(LimelightHelpers.getTV("limelight")){

				//System.out.println(result.getBotPose2d());
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
	public void runFollowTag(double x, double y, double area, double diagonalDistance,double floorDistance){
		for (SwerveModule module : modules){
			module.drivePID(diagonalDistance, 40);
			//module.turnPID();
			
		}
	}
	
	
	public void getOffsets() {
		for (SwerveModule module : modules) module.fixOffset();
	}

	public Command printOffsets() {
		return new InstantCommand(this::getOffsets, this);
	}

	
	

	public void initShuffleboard(){
		Shuffleboard.getTab("swerve").add(field);
	}
	
	public void init(Pose2d init_pose){
		pose_est = new SwerveDrivePoseEstimator(
			kinematics,
			imu.yaw(),
			getPositions(),
			init_pose,
			VecBuilder.fill(0.1, 0.1, 0.1),
			VecBuilder.fill(0.1, 0.1, 0.1));
		
	}
	
}
	
