package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConfiguration;


import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Constants.SwerveConstants.dtSeconds;
import static frc.robot.Constants.Constants.robotLength;
import static frc.robot.Constants.Constants.robotWidth;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class SwerveSubsystem extends SubsystemBase {
	private double slowSpeedMultiplier = 1;
	public double speedMultiplier;

	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(-robotLength / 2, -robotWidth / 2), // NW
			new Translation2d(-robotLength / 2, robotWidth / 2), // NE
			new Translation2d(robotLength / 2, robotWidth / 2), // SE
			new Translation2d(robotLength / 2, -robotWidth / 2) // SW
			);

	private SwerveModule[] modules = new SwerveModule[] {
		new SwerveModule(SwerveModuleConfiguration.NW, "NW"),
		new SwerveModule(SwerveModuleConfiguration.NE, "NE"),
		new SwerveModule(SwerveModuleConfiguration.SE, "SE"),
		new SwerveModule(SwerveModuleConfiguration.SW, "SW"),
	};

	double[] states = new double[8];



	public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] pos = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) pos[i] = modules[i].getPosition();
        return pos;
    }

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
		  states[i] = modules[i].getMeasuredState();
		}
		return states;
	  }
	public void setUpPathPlanner() {
		AutoBuilder.configureHolonomic(
			this::getCurrentPose, 
			this::resetOdom, 
			this::getChassisSpeedsAuto, 
			this::driveRobotRelative, 
			Constants.PATH_FOLLOWER_CONFIG,
			() -> {
				// Boolean supplier that controls when the path will be mirrored for the red alliance
				// This will flip the path being followed to the red side of the field.
				// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

				return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
			},
			this
    );
	}
	
	public ChassisSpeeds getChassisSpeedsAuto() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	public ChassisSpeeds getChassisSpeedsTeleop(double xVelocity, double yVelocity, double rotationalVelocity) {
		return ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, POSE_ESTIMATOR.getEstimatedPose().getRotation()),0.02);
	}

	public void driveRobotRelative(ChassisSpeeds speeds) {
		ChassisSpeeds targetSpeeds = new ChassisSpeeds(-speeds.vxMetersPerSecond,-speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond);

		SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);
		for (int i = 0; i < modules.length; i++) {
			modules[i].setState(states[i]);
		}
	}

	public Pose2d getCurrentPose() {
		return POSE_ESTIMATOR.getEstimatedPose();
	}

	public void resetOdom(Pose2d pose) {
		POSE_ESTIMATOR.resetOdomGivenPose2d(pose);
	  }

	public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {
		ChassisSpeeds speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, POSE_ESTIMATOR.getEstimatedPose().getRotation());
	
		speeds = ChassisSpeeds.discretize(speeds, dtSeconds);
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
		for (int i = 0; i < modules.length; i++) {
			modules[i].setState(states[i]);
		}
	}
	/*
	* The two commands below are commands to follow a single path for two different "phases", teleop and auto.
	* The command that I tested was the teleop one, but to actually get the right velocities to use them with a pigeon and gyro
	*  is too much work, so I made a different command.
	* followPathCommandTeleop accepts an x,y, and rotationalVelocity that I tried to get using the Pigeon, but it doesn't seem to
	* be accurate. So I created a new getChassisSpeedsAuto command that passes in module states (getModuleStates())
	* in place of the getChassisSpeeds method that takes in the velocities. Testing is pending, but I just thought that it might be useful.
	*
	* AutoBuilder has also been configured in the SwerveSubsystem constructor class, so we can work with autos now.
	* */
	

	public Command followPathCommandAuto(String pathName) {
		PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

		return new FollowPathHolonomic(
				path,
				this::getCurrentPose, // Robot pose supplier
				this::getChassisSpeedsAuto,// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
						new PIDConstants(0, 0.0, 0.0), // Translation PID constants
						new PIDConstants(0, 0.0, 0.0), // Rotation PID constants
						SwerveConstants.maxSpeed, // Max module speed, in m/s
						Units.feetToMeters(1), // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig() // Default path replanning config. See the API for the options here
				),
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this // Reference to this subsystem to set requirements
		);
	}

	public Command slowModeOn(){
		return this.runOnce(
				()-> slowSpeedMultiplier = 0.5
		);
	}

	public Command slowModeOff(){
		return this.runOnce(
				()-> slowSpeedMultiplier = 1
		);
	}

	public double getSlowSpeedMultiplier() {
		return slowSpeedMultiplier;
	}
	@Override
	public void periodic() {
		speedMultiplier = getSlowSpeedMultiplier();

		for (int i = 0; i < 4; i++) states[i * 2 + 1] = modules[i].getTargetState().speedMetersPerSecond;
		for (int i = 0; i < 4; i++)
			states[i * 2] = modules[i].getTargetState().angle.getRadians();
		Logger.recordOutput("Target States", states);
		for (int i = 0; i < 4; i++) states[i * 2 +1] = modules[i].getMeasuredState().speedMetersPerSecond;
		for (int i = 0; i < 4; i++)
			states[i * 2] = modules[i].getMeasuredState().angle.getRadians();
		Logger.recordOutput("Measured States", states);
		// //if statment is so that the telop wont run if selfdrive is on.
		for (SwerveModule module : modules) {
			module.periodic();
		}
	
		
	}

	public Command printOffsets() {
		return new InstantCommand(() -> {
			for (SwerveModule module : modules){
				module.printOffset();
			}
		}, this);
	}

	public SwerveDriveKinematics getKinematics(){
		return kinematics;
	}


}
	
