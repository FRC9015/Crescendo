package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

	private SwerveDrive swerveDrive;

	public SwerveSubsystem() {
		SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

		File directory = new File(Filesystem.getDeployDirectory(),"swerve");

        try {
			swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.maxSpeed);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
	}

	/**
	 * Command to drive the robot using translative values and heading as angular velocity.
	 *
	 * @param translationX     Translation in the X direction.
	 * @param translationY     Translation in the Y direction.
	 * @param angularRotationX Rotation of the robot to set
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
		return run(() -> {
			// Make the robot move
			swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
							translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
					angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
					true,
					false);
		});
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
	
