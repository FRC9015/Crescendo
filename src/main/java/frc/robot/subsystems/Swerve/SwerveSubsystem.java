package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
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


public class SwerveSubsystem extends SubsystemBase {
	private SwerveDrive swerveDrive;

	public SwerveSubsystem() {
		SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

		File directory = new File(Filesystem.getDeployDirectory(),"swerve");

        try {
			swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.maxSpeed);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

		swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
		swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
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
			swerveDrive.drive(new Translation2d((Math.pow(translationX.getAsDouble()*0.8, 3) * swerveDrive.getMaximumVelocity()),
							(Math.pow(translationY.getAsDouble()*0.8, 3) * swerveDrive.getMaximumVelocity())),
					Math.pow(angularRotationX.getAsDouble()*0.91, 3) * swerveDrive.getMaximumAngularVelocity(),
					true,
					false);
		});
	}
	@Override
	public void periodic() {
	}
}
	
