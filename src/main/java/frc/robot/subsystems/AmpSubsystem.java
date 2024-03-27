package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ShooterConstants;

public class AmpSubsystem extends SubsystemBase{
        
    private final CANSparkFlex ampMotorTop = new CANSparkFlex(ShooterConstants.ampShooterMotor1ID,
            MotorType.kBrushless);
    private final CANSparkFlex ampMotorBottom = new CANSparkFlex(ShooterConstants.ampShooterMotor2ID,
            MotorType.kBrushless);

    public AmpSubsystem(){
        ampMotorTop.setSmartCurrentLimit(30);
        ampMotorBottom.setSmartCurrentLimit(30);

        ampMotorTop.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 1000);
        ampMotorTop.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000);
        ampMotorBottom.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 1000);
        ampMotorBottom.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000);
    }

    public Command ampIntake(){
        return this.startEnd(
                this::setAmpIntakeSpeeds,
                this::stopAmpShooterMotorSpeeds
        );
    }

    public Command shootNoteToAmp() {
        return this.startEnd(
                this::setAmpShooterMotorSpeeds,
                this::stopAmpShooterMotorSpeeds
        );
    }

    public Command stopAmp(){
        return this.runOnce(this::stopAmpShooterMotorSpeeds);
    }

    public void setAmpShooterMotorSpeeds() {
        double motorSpeed = 0.8;// needs to be tuned
        ampMotorTop.set(-motorSpeed);
        ampMotorBottom.set(motorSpeed);
    }

    public void stopAmpShooterMotorSpeeds() {
        ampMotorTop.stopMotor();
        ampMotorBottom.stopMotor();
    }

    public void setAmpIntakeSpeeds(){
        double motorSpeed = 0.8; //needs to be tuned
        ampMotorTop.set(motorSpeed);
        ampMotorBottom.set(motorSpeed);
    }
}
