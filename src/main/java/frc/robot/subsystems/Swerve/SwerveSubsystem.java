package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.SwerveConstants.maxSpeed;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();
	public double speedMultiplier = 1;

    
    double[] states = new double[8];
    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        
        CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);
        for(int i = 0; i<4; i++){
            //getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
            getModule(i).getSteerMotor().getConfigurator().apply(limitsConfigs);
            getModule(i).getDriveMotor().getConfigurator().apply(limitsConfigs);

        }
        setUpPathPlanner();
       if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void drive(double xVelocity, double yVelocity, double rotationalVelocity){
        
        final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(rotationalVelocity);

            this.setControl(driveRequest);
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds(){
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
    public Pose2d getPose(){
        return getState().Pose;
    }

    public void setUpPathPlanner(){
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations){
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::seedFieldRelative, 
            this::getRobotRelativeChassisSpeeds, 
           (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),
            new HolonomicPathFollowerConfig(
                new PIDConstants(6.25,0,0),
                new PIDConstants(0.5,0,0), 
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()), 
            () -> RobotContainer.IsRed(),
            this);
    }

    public Command slowModeOn(){
		return this.runOnce(
				()-> speedMultiplier = 0.2
		);
	}

	public Command slowModeOff(){
		return this.runOnce(
				()-> speedMultiplier = 1
		);
	}

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) states[i*2 + 1] = getModule(i).getCurrentState().speedMetersPerSecond;
        
		for (int i = 0; i < 4; i++)
			states[i * 2] = getModule(i).getTargetState().angle.getRadians();
		Logger.recordOutput("Target States", states);
		for (int i = 0; i < 4; i++) states[i * 2 +1] = getModule(i).getCurrentState().speedMetersPerSecond;
		for (int i = 0; i < 4; i++)
			states[i * 2] = getModule(i).getCurrentState().angle.getRadians();
		Logger.recordOutput("Measured States", states);
    }
}
