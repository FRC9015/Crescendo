package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class AmpSubsystem extends SubsystemBase{
        
    private final SparkFlex ampMotorTop = new SparkFlex(ShooterConstants.ampShooterMotor1ID,
            MotorType.kBrushless);
    private final SparkFlex ampMotorBottom = new SparkFlex(ShooterConstants.ampShooterMotor2ID,
            MotorType.kBrushless);

    public AmpSubsystem(){

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
        double motorSpeed = 1; //needs to be tuned
        ampMotorTop.set(motorSpeed);
        ampMotorBottom.set(motorSpeed);
    }
}
