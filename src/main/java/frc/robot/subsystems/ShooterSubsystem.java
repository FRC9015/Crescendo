
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkFlex speakerMotorTop = new CANSparkFlex(ShooterConstants.speakerShooterMotorTopID,
            MotorType.kBrushless);
    private final CANSparkFlex speakerMotorBottom = new CANSparkFlex(ShooterConstants.speakerShooterMotor2ID,
            MotorType.kBrushless);

    private final CANSparkFlex ampShooterMotorTop = new CANSparkFlex(ShooterConstants.ampShooterMotor1ID,
            MotorType.kBrushless);
    private final CANSparkFlex ampShooterMotorBottom = new CANSparkFlex(ShooterConstants.ampShooterMotor2ID,
            MotorType.kBrushless);

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
                this::setSpeakerShooterMotorSpeeds,
                this::stopSpeakerShooterMotors);
    }

    public Command autoShootNoteToSpeaker() {
        return new SequentialCommandGroup(
                new InstantCommand(this::setSpeakerShooterMotorSpeeds),
                new WaitCommand(3),
                new InstantCommand(this::stopSpeakerShooterMotors));
    }

    public Command shootNoteToAmp() {
        return this.startEnd(
                this::setAmpShooterMotorSpeeds,
                this::stopAmpShooterMotorSpeeds
                );
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

    public void setSpeakerShooterMotorSpeeds(){
        double motorSpeed = 0.65;//needs to be tuned
        speakerMotorTop.set(0.60);
        speakerMotorBottom.set(0.60);
    }

    public void stopSpeakerShooterMotors() {
        speakerMotorTop.stopMotor();
        speakerMotorBottom.stopMotor();
    }

    private void setAmpShooterMotorSpeeds() {
        double motorSpeed = 0.5;// needs to be tuned
        ampShooterMotorTop.set(motorSpeed);
        ampShooterMotorBottom.set(motorSpeed);
    }

    public void stopAmpShooterMotorSpeeds() {
        ampShooterMotorTop.stopMotor();
        ampShooterMotorBottom.stopMotor();
    }

    public void setAmpIntakeSpeeds(){
        double motorSpeed = 0.9; //needs to be tuned
        ampShooterMotorTop.set(motorSpeed);
        ampShooterMotorBottom.set(motorSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
