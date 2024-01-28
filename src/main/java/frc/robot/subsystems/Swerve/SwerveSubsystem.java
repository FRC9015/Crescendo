package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.*;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import java.util.Map;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConfiguration;

import java.util.Map;

public class SwerveSubsystem extends SubsystemBase {

	private SimpleWidget speedMultiplierWidget = Shuffleboard.getTab("Drive")
		.add("Max Speed", 0.5)
		.withWidget(BuiltInWidgets.kNumberSlider)
		.withProperties(Map.of("min", 0, "max", 1)); // specify widget properties here
		
		
	private SimpleWidget angularMultiplierWidget = Shuffleboard.getTab("Drive")
		.add("Max Angular Speed", 0.5)
		.withWidget(BuiltInWidgets.kNumberSlider)
		.withProperties(Map.of("min", 0, "max", 1)); // specify widget properties here

		private double speedMultiplier; 
		private double angularMultiplier; 

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

	public ChassisSpeeds getChassisSpeeds(double xVelocity, double yVelocity, double rotationalVelocity) {
		return ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, POSE_ESTIMATOR.getEstimatedPose().getRotation()),0.02);
}
	public void drive(ChassisSpeeds speeds) {
	SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
		for (int i = 0; i < modules.length; i++) {
		modules[i].setState(states[i]);
}

	
}

		/**
	 * Get the closest angle between the given angles.
	 */
	public double closestAngle(double a, double b)
	{
			// get direction
			double dir = (b % 360.0) - (a % 360.0);

			// convert from -360 to 360 to -180 to 180
			if (Math.abs(dir) > 180.0)
			{
					dir = -(Math.signum(dir) * 360.0) + dir;
			}
			return dir;
	}
	public  Pose2d getCurrentPose(){
		return POSE_ESTIMATOR.getEstimatedPose();
	}

	public void resetOdom() {
		POSE_ESTIMATOR.resetOdometry();
	}

	public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {
		ChassisSpeeds speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, POSE_ESTIMATOR.getEstimatedPose().getRotation());
		speeds = ChassisSpeeds.discretize(speeds, 0.02);
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
		for (int i = 0; i < modules.length; i++) {
			modules[i].setState(states[i]);
		}
	}

	public Command followPathCommand(String pathName, double xVelocity, double yVelocity, double rotationalVelocity) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathHolonomic(
                path,
                this::getCurrentPose, // Robot pose supplier
				() -> this.getChassisSpeeds(xVelocity,yVelocity,rotationalVelocity),// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(3, 0.0, 0.0), // Rotation PID constants
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


	@Override
	public void periodic() {
		//if statment is so that the telop wont run if selfdrive is on.
		for (SwerveModule module : modules) {
			module.teleop();
		}
		speedMultiplier = speedMultiplierWidget.getEntry().get().getDouble()/100;
		angularMultiplier = angularMultiplierWidget.getEntry().get().getDouble()/100;
		POSE_ESTIMATOR.updatePoseEstimator();
		
	}

	public void getOffsets() {
		for (SwerveModule module : modules) module.fixOffset();
	}

	public Command printOffsets() {
		return new InstantCommand(this::getOffsets, this);
	}

	public double getSpeedMultiplier(){
		return speedMultiplier;
	}

	public double getAngularMultiplier(){
		return angularMultiplier;
	}

	public SwerveDriveKinematics getKinematics(){
		return kinematics;
	}
	
}
	
