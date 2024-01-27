package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class IMU {
	private Pigeon2 imu;

	public IMU() {
		imu = new Pigeon2(23);
	}

	public Rotation2d yaw() {
		return Rotation2d.fromDegrees(imu.getYaw().getValue());
	}

	public void zeroYaw() {
		imu.setYaw(0);
	}

	public double getHeading() {
        return Math.IEEEremainder(imu.getAngle(), 360);
    }

	public double getXVelocity() {
		//includes acceleration due to gravity
		return imu.getAccelerationX().getValueAsDouble();
	}

	public double getYVelocity() {
		return imu.getAccelerationY().getValueAsDouble();
	}

	public double getRotationalVelocity() {
		return imu.getAngularVelocityXDevice().getValueAsDouble() * (Math.PI/180);
	}

	public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
}
