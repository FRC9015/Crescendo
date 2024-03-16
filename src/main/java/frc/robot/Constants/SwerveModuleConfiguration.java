package frc.robot.Constants;


import edu.wpi.first.math.geometry.Rotation2d;

public enum SwerveModuleConfiguration {
	NW(34, 24, 14, 0.859131),
	NE(33, 23, 13, 0.340576),
	SW(32, 22, 12, 0.724121),
	SE(31, 21, 11, 0.913818);
	public int ENCODER, TURN_MOTOR, DRIVE_MOTOR;
	public Rotation2d offset;

	SwerveModuleConfiguration(int enc, int tm, int dm, double offset_rot) {
		ENCODER = enc;
		TURN_MOTOR = tm;
		DRIVE_MOTOR = dm;
		offset = Rotation2d.fromRotations(offset_rot);
	}
}
