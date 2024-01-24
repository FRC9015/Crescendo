package frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;

public enum SwerveModuleConfiguration {
	NW(27, 60, 10, 0.3447265625),
	NE(13, 16, 6, 0.32470703125),
	SW(39, 19, 17, 0.6 - 0.2495117),
	SE(34, 9, 8, 0.4 - 0.15332);
	public int ENCODER, TURN_MOTOR, DRIVE_MOTOR;
	public Rotation2d offset;

	SwerveModuleConfiguration(int enc, int tm, int dm, double offset_rot) {
		ENCODER = enc;
		TURN_MOTOR = tm;
		DRIVE_MOTOR = dm;
		offset = Rotation2d.fromRotations(offset_rot);
	}
}
