
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase {
    private final DigitalOutput speakerSensor = new DigitalOutput(0);

    private final double motorMaxFreeSpeed = 6784;
    private boolean shooterIsRunning = false;
    private boolean idleMode = false;
    private PIDController speakerPIDTop = new PIDController((10/motorMaxFreeSpeed),0,0);
    private PIDController speakerPIDBottom = new PIDController((10/motorMaxFreeSpeed),0,0);
    private final CANSparkFlex speakerMotorTop = new CANSparkFlex(ShooterConstants.speakerShooterMotorTopID,
            MotorType.kBrushless);
    private final CANSparkFlex speakerMotorBottom = new CANSparkFlex(ShooterConstants.speakerShooterMotor2ID,
            MotorType.kBrushless);


    RelativeEncoder speakerMotorTopEncoder = speakerMotorTop.getEncoder();
    RelativeEncoder speakerMotorBottomEncoder = speakerMotorBottom.getEncoder();

    public ShooterSubsystem() {
        speakerMotorTop.setSmartCurrentLimit(40);
        speakerMotorBottom.setSmartCurrentLimit(40);
        speakerPIDTop.setTolerance(200);
        speakerPIDBottom.setTolerance(200);
    }


    public double motorVelocity(RelativeEncoder encoder){
        return encoder.getVelocity();
    }

    public Command shootNoteToSpeaker() {
        return this.startEnd(
                this::setSpeakerShooterMotorSpeedsSubWoofer,
                this::setIdleShooterSpeeds);
    }

    public Command autoShootNoteToSpeaker(AmpSubsystem amp) {
        return new SequentialCommandGroup(
                new InstantCommand(this::setSpeakerShooterMotorSpeedsSubWoofer),
                new WaitCommand(1.5),
                new InstantCommand(amp::setAmpIntakeSpeeds),
                new WaitCommand(0.5),
                new InstantCommand(amp::stopAmpShooterMotorSpeeds),
                new InstantCommand(this::setIdleShooterSpeeds));
    }

    public Command autoShootNoteLimelight(AmpSubsystem amp) {
        return new SequentialCommandGroup(
                new InstantCommand(this::setSpeakerShooterMotorSpeeds),
                new WaitCommand(0.5),
                new InstantCommand(amp::setAmpIntakeSpeeds),
                new WaitCommand(0.3),
                new InstantCommand(amp::stopAmpShooterMotorSpeeds),
                new InstantCommand(this::setIdleShooterSpeeds));
    }

    public Command revShooter(){
        return this.runOnce(this::revShooterMotors);
    }


    public Command stopShooter() {
        return this.runOnce(
                this::stopSpeakerShooterMotors);
    }


    public Command shooterBackward(){
        return this.startEnd(
                this::backwardsShooter,
                this::setIdleShooterSpeeds);
    }


    public Command autoBackwardShooter(){
        return new SequentialCommandGroup(
                new InstantCommand(this::backwardsShooter),
                new WaitCommand(3),
                new InstantCommand(this::setIdleShooterSpeeds));
    }

    public Command enableIdleMode(){
        return this.runOnce(this::setIdleShooterSpeeds);
    }

    public void setIdleShooterSpeeds() {
        speakerPIDTop.setSetpoint(0.2 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.2 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = true;
    }

    public void setSpeakerShooterMotorSpeedsSubWoofer(){
        speakerPIDTop.setSetpoint(0.5 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.5 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;
    }

    public void setSpeakerShooterMotorSpeeds(){
        speakerPIDTop.setSetpoint(0.8 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.6 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;

    }

    public void stopSpeakerShooterMotors() {
        speakerMotorTop.stopMotor();
        speakerMotorBottom.stopMotor();
        shooterIsRunning = false;
        idleMode = false;
    }


    private void backwardsShooter(){
        speakerPIDTop.setSetpoint(-0.4 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(-0.4 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;
    }

    public void revShooterMotors(){
        speakerPIDTop.setSetpoint(0.6 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.4 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;
    }
    public boolean getShooterSensor() {
        return speakerSensor.get();
    }

    public boolean shooterIsReady(){
        return (speakerPIDTop.atSetpoint() && speakerPIDBottom.atSetpoint() && shooterIsRunning && !idleMode);
    }


    @Override
    public void periodic() {
        if (shooterIsRunning) {
            double setPointTop = MathUtil.clamp(speakerPIDTop.calculate(motorVelocity(speakerMotorTopEncoder)),-10,10);
            double setPointBottom = MathUtil.clamp(speakerPIDBottom.calculate(motorVelocity(speakerMotorBottomEncoder)),-10,10);

            speakerMotorTop.setVoltage(setPointTop);
            speakerMotorBottom.setVoltage(setPointBottom);
        }
        SmartDashboard.putBoolean("Shooter Sensor", speakerSensor.get());
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
