package frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;

public enum SwerveModuleConfiguration {
	NW(33, 23, 13, 0.3447265625), //TODO Change this ID Back to 15 on The Robot and Update it Here
	NE(31, 21, 11, 0.32470703125),
	SW(34, 24, 14, 0.6 - 0.2495117),
	SE(32, 22, 12, 0.4 - 0.15332);
	public int ENCODER, TURN_MOTOR, DRIVE_MOTOR;
	public Rotation2d offset;

	SwerveModuleConfiguration(int enc, int tm, int dm, double offset_rot) {
		ENCODER = enc;
		TURN_MOTOR = tm;
		DRIVE_MOTOR = dm;
		offset = Rotation2d.fromRotations(offset_rot);
	}
}
