package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
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
import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {
	public void velocityGraphUpdate(double xVelocity, double yVelocity) {
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

		File directory = new File(Filesystem.getDeployDirectory(), "swerve");

		try {
			swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.maxSpeed);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}

		swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
		swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
		setupPathPlanner();
	}

	public SwerveDriveKinematics getKinematics() {
		return swerveDrive.kinematics;
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
	 * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
	 * keep working.
	 *
	 * @param initialHolonomicPose The pose to set the odometry to
	 */
	public void resetOdometry(Pose2d initialHolonomicPose) {
		swerveDrive.resetOdometry(initialHolonomicPose);
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by odometry.
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
	 * Post the trajectory to the field.
	 *
	 * @param trajectory The trajectory to post.
	 */
	public void postTrajectory(Trajectory trajectory) {
		swerveDrive.postTrajectory(trajectory);
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
	 */
	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	/**
	 * Sets the drive motors to brake/coast mode.
	 *
	 * @param brake True to set motors to brake mode, false for coast.
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	/**
	 * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
	 * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
	 *
	 * @return The yaw angle
	 */
	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

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
						new PIDConstants(0, 01, 0.1), // Translation PID constants
						new PIDConstants(0.01, 0.0, 0.001), // Rotation PID constants
						SwerveConstants.maxSpeed, // Max module speed, in m/s
						swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig() // Default path replanning config. See the API for the options here
				),
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
					var alliance = DriverStation.getAlliance();
					return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
				},
				this // Reference to this subsystem to set requirements
		);
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
			swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
							Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
					Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
					true,
					false);
		});
	}

	public Command followPathCommandManual(String fileString) {
		PathPlannerPath path = PathPlannerPath.fromPathFile(fileString);

		return new FollowPathHolonomic(
				path,
				this::getPose,
				this::getRobotVelocity,
				this::setChassisSpeeds,
				new HolonomicPathFollowerConfig(
						new PIDConstants(0, 0.1, 0.1), // Translation PID constants
						new PIDConstants(0.01, 0.0, 0.001), // Rotation PID constants
						SwerveConstants.maxSpeed, // Max module speed, in m/s
						swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig() // Default path replanning config. See the API for the options here
				),
				() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Blue),
				this);
	}

	//The below code and command auto-generates a path to a specific point using PathPlanner. This will be useful for autonomous routines when picking up notes
	Pose2d targetPose = new Pose2d(2.5, 5, Rotation2d.fromDegrees(180));

	// Create the constraints to use while pathfinding
	PathConstraints constraints = new PathConstraints(
			3.0, 3.0,
			Units.degreesToRadians(540), Units.degreesToRadians(720));
	Command pathfindingCommandtoFirstNote = AutoBuilder.pathfindToPose(
			targetPose,
			constraints,
			0.0, // Goal end velocity in meters/sec
			0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
	);

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
	
