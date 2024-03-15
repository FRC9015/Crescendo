package frc.robot.Constants;


import edu.wpi.first.math.geometry.Rotation2d;

public enum SwerveModuleConfiguration {
	NW(31, 21, 11, 148.7109375),
	NE(32, 22, 12, 80.068359375),
	SW(33, 23, 13, 301.025390625),
	SE(34, 24, 14, 129.814453125);
	public int ENCODER, TURN_MOTOR, DRIVE_MOTOR;
	public Rotation2d offset;

	SwerveModuleConfiguration(int enc, int tm, int dm, double offset_rot) {
		ENCODER = enc;
		TURN_MOTOR = tm;
		DRIVE_MOTOR = dm;
		offset = Rotation2d.fromRotations(offset_rot);
	}
}
