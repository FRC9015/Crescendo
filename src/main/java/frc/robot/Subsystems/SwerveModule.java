package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveModuleConfiguration;

public class SwerveModule {
	public CANSparkFlex turn, drive;
	public CANcoder encoder;
	public Rotation2d encoder_offset;

	public RelativeEncoder driveEncoder;
	public SwerveModuleState targState;

	private PIDController drivePID, turnPPID;
	private String name;
	private double kV = 3;

	public SwerveModule(SwerveModuleConfiguration config, String nameString) {
		turn = new CANSparkFlex(config.TURN_MOTOR, MotorType.kBrushless);
		drive = new CANSparkFlex(config.DRIVE_MOTOR, MotorType.kBrushless);
		name = nameString;
		encoder = new CANcoder(config.ENCODER);

		drivePID = new PIDController(3, 0, 0);
		turnPPID = new PIDController(2, 0, 0);
		turnPPID.enableContinuousInput(-PI, PI);
		encoder_offset = config.offset;
		drive.restoreFactoryDefaults();
		turn.restoreFactoryDefaults();

		drive.setCANTimeout(250);
		turn.setCANTimeout(250);

		driveEncoder = drive.getEncoder();
		driveEncoder.setPosition(0.0);

		drive.setSmartCurrentLimit(40);
		turn.setSmartCurrentLimit(30);
		drive.enableVoltageCompensation(12.0);
		turn.enableVoltageCompensation(12.0);

		drive.setCANTimeout(0);
		turn.setCANTimeout(0);

		drive.burnFlash();
		turn.burnFlash();
	}

	public Rotation2d getDirection() {
		return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue())
				.minus(encoder_offset);
	}

	public void setState(SwerveModuleState state) {
		this.targState = SwerveModuleState.optimize(state, getDirection());
	}

	public void fixOffset() {
		System.out.println("ERROR Offset for Cancoder: " + this.name + " is: "
				+ getDirection().plus(encoder_offset).getRotations());
	}

	public void periodic() {
		if (targState == null) return;
		double curr_velocity =
				Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / gearRatio * wheelRatio;
		double target_vel = Math.abs(Math.cos((getDirection().getRadians() - targState.angle.getRadians())))
				* targState.speedMetersPerSecond;
		if (name == "NW") {
			// System.out.println("Module: " + this.name + " Target_angle: " + targState.angle.getRadians() +
			// "current_angle: " + getDirection().getRadians());
		}
		drive.setVoltage(drivePID.calculate(curr_velocity, target_vel) + target_vel * kV);
		turn.setVoltage(turnPPID.calculate(getDirection().getRadians(), targState.angle.getRadians()));
	}
}
