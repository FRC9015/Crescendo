package frc.robot.subsystems;

import static frc.robot.Constants.Constants.PigeonConstants.*;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon {
	private Pigeon2 pigeon;

	public Pigeon() {
		pigeon = new Pigeon2(pigeonID);
	}

	public double getYaw() {
		return pigeon.getYaw().getValue();
	}

	public Rotation2d getYawAsRotation2d(){ return Rotation2d.fromDegrees(pigeon.getYaw().getValue()); }

	public void zeroYaw() {
		pigeon.setYaw(0);
	}

	public void resetYaw(double angle){ pigeon.setYaw(angle); }
}
