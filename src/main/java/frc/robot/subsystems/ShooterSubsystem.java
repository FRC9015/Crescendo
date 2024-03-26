package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.BangBangController;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final double motorMaxFreeSpeed = 6784;
    private boolean shooterIsRunning = false;
    private BangBangController speakerPIDTop = new BangBangController();
    private BangBangController speakerPIDBottom = new BangBangController();
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
        
   
        speakerMotorTop.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 1000);
        speakerMotorTop.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000);
        speakerMotorBottom.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 1000);
        speakerMotorBottom.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000);
     
    }

    public double motorVelocity(RelativeEncoder encoder){
        return encoder.getVelocity();
    }

    public Command shootNoteToSpeaker() {
        return this.startEnd(
                this::setSpeakerShooterMotorSpeedsSubWoofer,
                this::stopSpeakerShooterMotors);
    }

    public Command autoShootNoteToSpeaker(AmpSubsystem amp) {
        return new SequentialCommandGroup(
                new InstantCommand(this::setSpeakerShooterMotorSpeedsSubWoofer),
                new WaitCommand(1.5),
                new InstantCommand(amp::setAmpIntakeSpeeds),
                new WaitCommand(0.5),
                new InstantCommand(amp::stopAmpShooterMotorSpeeds),
                new InstantCommand(this::stopSpeakerShooterMotors));
    }

    public Command autoShootNoteLimelight(AmpSubsystem amp) {
        return new SequentialCommandGroup(
                new InstantCommand(this::setSpeakerShooterMotorSpeeds),
                new WaitCommand(0.5),
                new InstantCommand(amp::setAmpIntakeSpeeds),
                new WaitCommand(0.3),
                new InstantCommand(amp::stopAmpShooterMotorSpeeds),
                new InstantCommand(this::stopSpeakerShooterMotors));
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
                this::stopSpeakerShooterMotors);
    }


    public Command autoBackwardShooter(){
        return new SequentialCommandGroup(
            new InstantCommand(this::backwardsShooter),
            new WaitCommand(3),
            new InstantCommand(this::stopSpeakerShooterMotors));
}
    
    public void setSpeakerShooterMotorSpeedsSubWoofer(){
        speakerPIDTop.setSetpoint(0.7 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.5 * motorMaxFreeSpeed);
        shooterIsRunning=true;
    }

    public void setSpeakerShooterMotorSpeeds(){
        speakerPIDTop.setSetpoint(0.8 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.6 * motorMaxFreeSpeed);
        shooterIsRunning=true;
    }

    public void revShooterMotors(){
        speakerPIDTop.setSetpoint(0.6 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.4 * motorMaxFreeSpeed);
        shooterIsRunning=true;
    }

    public void stopSpeakerShooterMotors() {
        speakerMotorTop.stopMotor();
        speakerMotorBottom.stopMotor();
        shooterIsRunning=false;
    }

    private void backwardsShooter(){
        speakerPIDTop.setSetpoint(-0.8 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(-0.8 * motorMaxFreeSpeed);
        shooterIsRunning=true;
    }

    public boolean shooterIsReady(){
        return (speakerPIDTop.atSetpoint() && speakerPIDBottom.atSetpoint() && shooterIsRunning);
    }

    @Override
    public void periodic() {
        if (shooterIsRunning) {
            speakerMotorTop.setVoltage(speakerPIDTop.calculate(motorVelocity(speakerMotorTopEncoder)) * 10);
            speakerMotorBottom.setVoltage(speakerPIDBottom.calculate(motorVelocity(speakerMotorBottomEncoder)) * 10);
        }
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
