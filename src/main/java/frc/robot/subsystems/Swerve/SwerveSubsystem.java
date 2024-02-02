package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.*;
import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import org.littletonrobotics.junction.Logger;
import java.util.Map;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConfiguration;

public class SwerveSubsystem extends SubsystemBase {
	public void velocityGraphUpdate(double xVelocity, double yVelocity){
		SmartDashboard.putNumber("xVelocity Graph", xVelocity);
		SmartDashboard.putNumber("yVelocity Graph", yVelocity);
	}
	private SimpleWidget speedMultiplierWidget = Shuffleboard.getTab("Drive")
			.add("Max Speed", .5)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 1));

	private SimpleWidget angularMultiplierWidget = Shuffleboard.getTab("Drive")
			.add("Max Angular Speed", .5)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 1));

	private double speedMultiplier;
	private double angularMultiplier;
	
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
	public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {
		ChassisSpeeds speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, POSE_ESTIMATOR.getEstimatedPose().getRotation());
				speeds = ChassisSpeeds.discretize(speeds, dtSeconds);
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
		for (int i = 0; i < modules.length; i++) {
			modules[i].setState(states[i]);
		}
	}


	@Override
	public void periodic() {
		//if statment is so that the telop wont run if selfdrive is on.
		for (SwerveModule module : modules) {
			module.teleop();
		}
		//TODO: Add all data visualization to one subsystem
		speedMultiplier = speedMultiplierWidget.getEntry().get().getDouble(); 
		angularMultiplier = angularMultiplierWidget.getEntry().get().getDouble();
	
	}

	public void getOffsets() {
		for (SwerveModule module : modules) module.fixOffset();
	}

	public Command printOffsets() {
		return new InstantCommand(this::getOffsets, this);
	}

	public SwerveDriveKinematics getKinematics(){
		return kinematics;
	}

	public double getSpeedMultiplier(){
		return speedMultiplier;
	}

	public double getAngularMultiplier(){
		return angularMultiplier;
	}
}
	
