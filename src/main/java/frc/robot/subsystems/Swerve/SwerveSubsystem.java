package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;


public class SwerveSubsystem extends SubsystemBase {
	private SwerveDrive swerveDrive;

	public SwerveSubsystem() {
		SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.NONE;

		File directory = new File(Filesystem.getDeployDirectory(), "swerve");

		try {
			swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.maxSpeed);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}

		swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot via
																						// angle.
		swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
																																					// simulations since it causes discrepancies
																																					// not seen in real life.
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not
	 * need to be reset when calling this
	 * method. However, if either gyro angle or module position is reset, this must
	 * be called in order for odometry to
	 * keep working.
	 *
	 * @param initialHolonomicPose The pose to set the odometry to
	 */
	public void resetOdometry(Pose2d initialHolonomicPose) {
		swerveDrive.resetOdometry(initialHolonomicPose);
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by
	 * odometry.
	 *
	 * @return The robot's pose
	 */
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds Chassis Speeds to set.
	 */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

    /**
	 * Gets the current yaw angle of the robot, as reported by the swerve pose
	 * estimator in the underlying drivebase.
	 * Note, this is not the raw gyro reading, this may be corrected from calls to
	 * resetOdometry().
	 *
	 * @return The yaw angle
	 */

	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}

	public void setupPathPlanner() {
		AutoBuilder.configureHolonomic(
				this::getPose, // Robot pose supplier
				this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig(
						new PIDConstants(0, 0.1, 0.1), // Translation PID constants
						new PIDConstants(0.01, 0.0, 0.001), // Rotation PID constants
						SwerveConstants.maxSpeed, // Max module speed, in m/s
						swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig() // Default path planning config. See the API for the options here
				),
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
					var alliance = DriverStation.getAlliance();
					return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
				},
				this // Reference to this subsystem to set requirements
		);
	}

	/**
	 * Command to drive the robot using translation values and heading as angular
	 * velocity.
	 *
	 * @param translationX     Translation in the X direction.
	 * @param translationY     Translation in the Y direction.
	 * @param angularRotationX Rotation of the robot to set
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
			DoubleSupplier angularRotationX) {
		return run(() -> {

			//gi Make the robot move
			swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
					Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
					Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
					true,
					false);
		});
	}

}
