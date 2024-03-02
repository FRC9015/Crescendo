package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class ClimberSubsystem extends SubsystemBase {
    CANSparkFlex climberMotor1 = new CANSparkFlex(Constants.ClimberConstants.climberMotor1ID, MotorType.kBrushless);
    //TODO Add CANSparkFlex climberMotor2 = new CANSparkFlex(Constants.ClimberConstants.climberMotor2ID, MotorType.kBrushless);

    public ClimberSubsystem() {
        //TODO add climberMotor2.follow(climberMotor1);
    }

    public Command goUp(){
        return this.startEnd(
                this::setMotorSpeedsUp,
                this::stopClimberMotorSpeeds
        );
    }

    public Command goDown(){
        return this.startEnd(
                this::setMotorSpeedsDown,
                this::stopClimberMotorSpeeds
        );
    }

    private void setMotorSpeedsUp(){
        double motorSpeed = 0.3;
            climberMotor1.set(motorSpeed);
    }

    private void setMotorSpeedsDown() {
        double motorSpeed = -0.3;
        climberMotor1.set(motorSpeed);
    }

    private void stopClimberMotorSpeeds(){
        climberMotor1.stopMotor();
    }
}
