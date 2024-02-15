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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.Map;

import static frc.robot.Constants.Constants.SwerveConstants.dtSeconds;
import static frc.robot.Constants.Constants.robotLength;
import static frc.robot.Constants.Constants.robotWidth;

public class SwerveSubsystem extends SubsystemBase {
	public void velocityGraphUpdate(double xVelocity, double yVelocity){
		SmartDashboard.putNumber("xVelocity Graph", xVelocity);
		SmartDashboard.putNumber("yVelocity Graph", yVelocity);
	}
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

	public SwerveSubsystem() {
		SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

		File directory = new File(Filesystem.getDeployDirectory(),"swerve");

        try {
            SwerveDrive swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.maxSpeed);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
	}

	public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {
	}

	@Override
	public void periodic() {
		//TODO: Add all data visualization to one subsystem
		speedMultiplier = speedMultiplierWidget.getEntry().get().getDouble(); 
		angularMultiplier = angularMultiplierWidget.getEntry().get().getDouble();
	}

	public double getSpeedMultiplier(){
		return speedMultiplier;
	}

	public double getAngularMultiplier(){
		return angularMultiplier;
	}

}
	
