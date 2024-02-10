package frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;

/* When determining which module is which, begin with the limelight side facing forward.
TOP LEFT: NW
TOP RIGHT: NE
BOTTOM RIGHT: SE
BOTTOM LEFT: SW
 */

public enum SwerveModuleConfiguration {
	NW(31, 21, 11, 0), //TODO Change this ID Back to 15 on The Robot and Update it Here
	NE(32, 22, 12, 0),
	SE(33, 23, 13, 0),
	SW(34, 24, 14, 0);
	public int ENCODER, TURN_MOTOR, DRIVE_MOTOR;
	public Rotation2d offset;

	SwerveModuleConfiguration(int enc, int tm, int dm, double offset_rot) {
		ENCODER = enc;
		TURN_MOTOR = tm;
		DRIVE_MOTOR = dm;
		offset = Rotation2d.fromRotations(offset_rot);
	}
}
