package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.gearRatio;
import static frc.robot.Constants.Constants.wheelRadius;
import static frc.robot.Constants.Constants.SwervePIDControllerConstants;
import static java.lang.Math.PI;

import com.revrobotics.CANSparkLowLevel;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveModuleConfiguration;

public class SwerveModule {
	private CANSparkFlex turn, drive;
	private CANcoder encoder;
	private Rotation2d encoderOffset;

	private RelativeEncoder driveEncoder;
	private SwerveModuleState targetState = new SwerveModuleState();;
	private PIDController drivePID, turnPPID;
	private String name;
	private double kV = 3;

	public SwerveModule(SwerveModuleConfiguration config, String nameString) {
		turn = new CANSparkFlex(config.TURN_MOTOR, MotorType.kBrushless);
		drive = new CANSparkFlex(config.DRIVE_MOTOR, MotorType.kBrushless);
		name = nameString;
		encoder = new CANcoder(config.ENCODER);

		drivePID = new PIDController(SwervePIDControllerConstants.driveP, SwervePIDControllerConstants.driveI, SwervePIDControllerConstants.driveD);
		turnPPID = new PIDController(SwervePIDControllerConstants.turnP, SwervePIDControllerConstants.turnI, SwervePIDControllerConstants.turnD);

		turnPPID.enableContinuousInput(-PI, PI);
		encoderOffset = config.offset;
		drive.restoreFactoryDefaults();
		turn.restoreFactoryDefaults();

		turn.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0,10000);
		turn.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1,10000);
		turn.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2,10000);
		turn.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3,10000);
		turn.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4,10000);
		turn.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,10000);
		turn.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6,10000);

		driveEncoder = drive.getEncoder();
		driveEncoder.setPosition(0.0);
		driveEncoder.setVelocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1)/gearRatio*wheelRadius);

		drive.setSmartCurrentLimit(40);
		turn.setSmartCurrentLimit(30);
		drive.enableVoltageCompensation(12.0);
		turn.enableVoltageCompensation(12.0);

		drive.setCANTimeout(0);
		turn.setCANTimeout(0);

		drive.burnFlash();
		turn.burnFlash();
	}

	public double getDirection() {
		Rotation2d turnAbsolutePosition = new Rotation2d(Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble()))
				.minus(encoderOffset);
		return turnAbsolutePosition.getRadians();
	}
	public SwerveModulePosition getPosition(){
		return new SwerveModulePosition(-getDriveDistance(), new Rotation2d(getDirection()));

	}
	public double getDriveDistance(){
		return driveEncoder.getPosition() * 2 * PI * wheelRadius / gearRatio;
	}
	public void setState(SwerveModuleState state) {
		this.targetState = SwerveModuleState.optimize(state,  new Rotation2d(getDirection()));
	}

	public SwerveModuleState getTargetState(){
		return targetState;
	}

	public SwerveModuleState getMeasuredState() {
		return new SwerveModuleState(driveEncoder.getVelocity(),  new Rotation2d(getDirection())); 
	}

	public void printOffset() {
		System.out.println("ERROR Offset for Cancoder: " + this.name + " is: "
				+  new Rotation2d(getDirection()).plus(encoderOffset).getRotations());
	}

	public void periodic() {
		if (targetState == null) return;
		Logger.recordOutput("swerveModules/" + name + "/targetstate", targetState);
		Logger.recordOutput("swerveModules/" + name + "/messuredstate", getMeasuredState());
		double curr_velocity =
				Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / gearRatio * wheelRadius;
		double target_vel = Math.abs(Math.cos(( new Rotation2d(getDirection()).getRadians() - targetState.angle.getRadians())))
				* targetState.speedMetersPerSecond;

		drive.setVoltage(drivePID.calculate(curr_velocity, target_vel) + target_vel * kV );
		turn.setVoltage(turnPPID.calculate( new Rotation2d(getDirection()).getRadians(), targetState.angle.getRadians()));

		Logger.recordOutput("swerveModules/" + name + "PIDCalculate", drivePID.calculate(curr_velocity, target_vel) + target_vel * kV);
		Logger.recordOutput("swerveModules/" + name + "/AppliedDriveOutput",drive.getAppliedOutput());
		Logger.recordOutput("swerveModules/" + name + "/AppliedDriveOutput",turn.getAppliedOutput());
	}

	
}
