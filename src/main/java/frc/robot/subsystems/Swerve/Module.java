package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Module {
	static final double ODOMETRY_FREQUENCY = 250.0;
	private final ModuleIO moduleIO;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private final int index;

	private final SimpleMotorFeedforward driveFeedFoward;
	private final PIDController driveFeedback;
	private final PIDController turnFeedback;
	private Rotation2d angleSetpoint = null;
	private Double speedSetpoint = null;
	private Rotation2d turnRelativeOffset = null;
	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

	public Module(ModuleIO io, int index){
		this.moduleIO = io;
		this.index = index;

		if (Robot.isSimulation()){
			driveFeedFoward = new SimpleMotorFeedforward(0.0, 0.13); // Sample Values for ks and kv
			driveFeedback = new PIDController(1.5, 0.0, 0.0); // Sample PID Values
			turnFeedback = new PIDController(1.5, 0.0, 0.0); // Sample PID Values
		}else{
			driveFeedFoward = new SimpleMotorFeedforward(0.0, 0.0);
			driveFeedback = new PIDController(0.0, 0.0, 0.0);
			turnFeedback = new PIDController(0.0, 0.0, 0.0);
		}

		turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
		setBrakeMode(false);
	}

	public void updateInputs(){
		moduleIO.updateInputs(inputs);
	}

	public void periodic(){
		Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

		if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0){
			turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
		}

		if (angleSetpoint != null){
			moduleIO.setTurnVoltage(
					turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians())
			);

			if (speedSetpoint != null){
				double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

				double velocityRadPerSec = adjustSpeedSetpoint / Constants.wheelRadius;
				moduleIO.setDriveVoltage(
						driveFeedFoward.calculate(velocityRadPerSec) +
								driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec)
				);
			}
		}

		int sampleCount = inputs.odometryTimestamps.length;
		odometryPositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			double positionMeters = inputs.odometryDrivePositionRad[i] * Constants.wheelRadius;
			Rotation2d angle =
					inputs.odometryTurnPositions[i].plus(
							turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());
			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}
	}

	public SwerveModuleState runSetpoint(SwerveModuleState state){
		var optimizedState = SwerveModuleState.optimize(state, getAngle());

		angleSetpoint = optimizedState.angle;
		speedSetpoint = optimizedState.speedMetersPerSecond;

		return optimizedState;
	}

	public void runCharacterization(double volts){
		angleSetpoint = new Rotation2d();

		moduleIO.setDriveVoltage(volts);
		speedSetpoint = null;
	}

	public void stop(){
		moduleIO.setTurnVoltage(0.0);
		moduleIO.setDriveVoltage(0.0);

		angleSetpoint = null;
		speedSetpoint = null;
	}

	public void setBrakeMode(boolean enabled){
		moduleIO.setDriveBrakeMode(enabled);
		moduleIO.setTurnBrakeMode(enabled);
	}

	public Rotation2d getAngle(){
		if (turnRelativeOffset == null) {
			return new Rotation2d();
		} else{
			return inputs.turnPosition.plus(turnRelativeOffset);
		}
	}

	public double getPositionMeters(){
		return inputs.drivePositionRad * Constants.wheelRadius;
	}

	public double getVelocityMetersPerSec(){
		return inputs.driveVelocityRadPerSec * Constants.wheelRadius;
	}

	public SwerveModulePosition getPosition(){
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	public SwerveModuleState getState(){
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	public SwerveModulePosition[] getOdometryPositions(){
		return odometryPositions;
	}

	public double[] getOdometryTimestamps(){
		return inputs.odometryTimestamps;
	}

	public double getCharacterizationVelocity(){
		return inputs.driveVelocityRadPerSec;
	}
}
