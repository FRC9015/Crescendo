package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private ShooterSubsystem shooter;


    private CANSparkFlex[] intakeMotors = new CANSparkFlex[]{
        new CANSparkFlex(IntakeConstants.intakeMotor1ID, MotorType.kBrushless),
        new CANSparkFlex(IntakeConstants.intakeMotor2ID, MotorType.kBrushless)


    };
    private CANSparkFlex handoffMotor = new CANSparkFlex(IntakeConstants.handoffMotorID, MotorType.kBrushless);


    public IntakeSubsystem(){
        for (CANSparkFlex motor:intakeMotors){
            motor.setSmartCurrentLimit(30);
        }
    }

    public Command intakeNote(){
        return this.startEnd(
           this::setIntakeMotorSpeeds,
           this::stopIntakeMotors
        );
    }
    public Command outtakeNote(){
        return this.startEnd(
                this::setReverseIntakeMotorSpeeds,
                this::stopIntakeMotors
        );
    }

    public Command stopIntake(){
        return new InstantCommand(
            this::stopIntakeMotors
        );
    }

    private void setIntakeMotorSpeeds(){
        double motorSpeed = -0.8;
        for (CANSparkFlex motor:intakeMotors){
            motor.set(motorSpeed);
        }
        handoffMotor.set(motorSpeed);
    }
    private void setReverseIntakeMotorSpeeds(){
        double motorSpeed = 0.8;
        for(CANSparkFlex motor:intakeMotors){
            motor.set(motorSpeed);
        }
        handoffMotor.set(motorSpeed);
    }
    private void stopIntakeMotors(){
        for (CANSparkFlex motor:intakeMotors){
            motor.set(0);
        }
        handoffMotor.stopMotor();
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


