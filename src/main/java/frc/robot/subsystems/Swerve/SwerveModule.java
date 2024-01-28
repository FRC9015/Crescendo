package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.*;
import static frc.robot.Constants.Constants.PIDConstants.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveModuleConfiguration;

public class SwerveModule {
	private CANSparkMax turn, drive;
	private CANcoder encoder;
	private Rotation2d encoderOffset;

	private RelativeEncoder driveEncoder;
	private SwerveModuleState targetState;

	private PIDController drivePID, turnPPID;
	private String name;
	private double kV = 3;

	public SwerveModule(SwerveModuleConfiguration config, String nameString) {
		turn = new CANSparkMax(config.TURN_MOTOR, MotorType.kBrushless);
		drive = new CANSparkMax(config.DRIVE_MOTOR, MotorType.kBrushless);
		name = nameString;
		encoder = new CANcoder(config.ENCODER);
		drivePID = new PIDController(driveP, driveI, driveD);
		turnPPID = new PIDController(turnP, turnI, turnD); 

        
		turnPPID.enableContinuousInput(-PI, PI);
		encoderOffset = config.offset;
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
				.minus(encoderOffset);
	}
	public SwerveModulePosition getPosition(){
		return new SwerveModulePosition(getDriveDistance(), getDirection());

	}
	public double getDriveDistance(){
		return driveEncoder.getPosition() / gearRatio * 2 * Math.PI * Units.inchesToMeters(2);
	}
	public void setState(SwerveModuleState state) {
		this.targetState = SwerveModuleState.optimize(state, getDirection());
	}

	public SwerveModuleState getTargetState(){
		return targetState;
	}

	public SwerveModuleState getMeasuredState() {
		return new SwerveModuleState((driveEncoder.getVelocity()/ gearRatio * 2 * Math.PI * Units.inchesToMeters(2)), getDirection()); 
	}
	
	public void fixOffset() {
		System.out.println("ERROR Offset for Cancoder: " + this.name + " is: "
				+ getDirection().plus(encoderOffset).getRotations());
	}

	public void teleop() {
		if (targetState == null) return;
		double curr_velocity =
				Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / gearRatio * wheelRatio;
		double target_vel = Math.abs(Math.cos((getDirection().getRadians() - targetState.angle.getRadians())))
				* targetState.speedMetersPerSecond;

		drive.setVoltage(drivePID.calculate(curr_velocity, target_vel) + target_vel * kV);
		turn.setVoltage(turnPPID.calculate(getDirection().getRadians(), targetState.angle.getRadians()));
	}
}
