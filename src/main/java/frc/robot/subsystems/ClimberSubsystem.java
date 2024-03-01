package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkFlex[] climberMotors = new CANSparkFlex[]{
            new CANSparkFlex(Constants.ClimberConstants.climberMotor1ID, MotorType.kBrushless),
            new CANSparkFlex(Constants.ClimberConstants.climberMotor2ID, MotorType.kBrushless)
    };

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
        for (CANSparkFlex motor:climberMotors){
            motor.set(motorSpeed);
        }
    }

    private void setMotorSpeedsDown(){
        double motorSpeed = -0.3;
        for (CANSparkFlex motor:climberMotors){
            motor.set(motorSpeed);
        }
    }

    private void stopClimberMotorSpeeds(){
        for (CANSparkFlex motor:climberMotors){
            motor.stopMotor();
        }
    }
}
