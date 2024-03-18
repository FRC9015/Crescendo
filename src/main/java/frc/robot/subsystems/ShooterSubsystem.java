
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private static final double speakerSpeed = 4400;
    private final CANSparkFlex speakerMotorTop = new CANSparkFlex(ShooterConstants.speakerShooterMotorTopID,
            MotorType.kBrushless);
    private final CANSparkFlex speakerMotorBottom = new CANSparkFlex(ShooterConstants.speakerShooterMotor2ID,
            MotorType.kBrushless);

    private final CANSparkFlex ampShooterMotorTop = new CANSparkFlex(ShooterConstants.ampShooterMotor1ID,
            MotorType.kBrushless);
    private final CANSparkFlex ampShooterMotorBottom = new CANSparkFlex(ShooterConstants.ampShooterMotor2ID,
            MotorType.kBrushless);

    RelativeEncoder speakerMotorTopEncoder = speakerMotorTop.getEncoder();

    public ShooterSubsystem() {
        speakerMotorTop.setSmartCurrentLimit(40);
        speakerMotorBottom.setSmartCurrentLimit(40);
        ampShooterMotorTop.setSmartCurrentLimit(30);
        ampShooterMotorBottom.setSmartCurrentLimit(30);
    }

    public Command shootNoteToSpeaker() {
        // TODO change this into a Sequential Command. We should set the first command
        // in the sequence to set the pivot angle.
        return this.startEnd(
                this::setSpeakerShooterMotorSpeedsSubWoofer,
                this::stopSpeakerShooterMotors);
    }

    public Command autoShootNoteToSpeaker() {
        return new SequentialCommandGroup(
                new InstantCommand(this::setSpeakerShooterMotorSpeedsSubWoofer),
                new WaitCommand(3),
                new InstantCommand(this::setAmpIntakeSpeeds),
                new WaitCommand(1),
                new InstantCommand(this::stopAmpShooterMotorSpeeds),
                new InstantCommand(this::stopSpeakerShooterMotors));
    }

    public Command shootNoteToAmp() {
        return this.startEnd(
                this::setAmpShooterMotorSpeeds,
                this::stopAmpShooterMotorSpeeds
                );
    }

    public Command autoShootNoteToAmp(){
        return new SequentialCommandGroup(
                new InstantCommand(this::setAmpShooterMotorSpeeds),
                new WaitCommand(1),
                new InstantCommand(this::stopAmpShooterMotorSpeeds));
    }

    public Command stopShooter() {
        return this.runOnce(
                this::stopSpeakerShooterMotors);
    }

    public Command ampIntake(){
        return this.startEnd(
                this::setAmpIntakeSpeeds,
                this::stopAmpShooterMotorSpeeds
        );
    }

    public Command shooterBackward(){
        return this.startEnd(
                this::backwardsShooter,
                this::stopSpeakerShooterMotors);
    }

    public Command autoAmpIntake(){
        return new SequentialCommandGroup(
                new InstantCommand(this::setAmpIntakeSpeeds),
                new WaitCommand(1),
                new InstantCommand(this::stopAmpShooterMotorSpeeds));
    }

    public Command autoBackwardShooter(){
        return new SequentialCommandGroup(
            new InstantCommand(this::backwardsShooter),
            new WaitCommand(3),
            new InstantCommand(this::stopSpeakerShooterMotors));
}
    
    public void setSpeakerShooterMotorSpeedsSubWoofer(){
        speakerMotorTop.set(0.7);
        speakerMotorBottom.set(0.5);
    }

    public void setSpeakerShooterMotorSpeeds(){
        speakerMotorTop.set(0.8);
        speakerMotorBottom.set(0.6);
    }

    public void stopSpeakerShooterMotors() {
        speakerMotorTop.stopMotor();
        speakerMotorBottom.stopMotor();
    }
    

    private void setAmpShooterMotorSpeeds() {
        double motorSpeed = 0.5;// needs to be tuned
        ampShooterMotorTop.set(-motorSpeed);
        ampShooterMotorBottom.set(motorSpeed);
    }

    private void stopAmpShooterMotorSpeeds() {
        ampShooterMotorTop.stopMotor();
        ampShooterMotorBottom.stopMotor();
    }

    private void setAmpIntakeSpeeds(){
        double motorSpeed = 0.8; //needs to be tuned
        ampShooterMotorTop.set(motorSpeed);
        ampShooterMotorBottom.set(motorSpeed);
    }

    private void backwardsShooter(){
        speakerMotorTop.set(-0.8);
        speakerMotorBottom.set(-0.80);
    }

    public double getSpeakerMotorRPM(){
        return speakerMotorTopEncoder.getVelocity();
    }

    public boolean shooterIsReady(){
        double motorSpeed = getSpeakerMotorRPM();
        SmartDashboard.putNumber("Shooter Speed", motorSpeed);
        return (speakerSpeed - motorSpeed) <= 100;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
