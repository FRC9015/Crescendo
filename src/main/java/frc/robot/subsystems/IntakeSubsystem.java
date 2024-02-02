package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private CANSparkMax[] intakeMotors = new CANSparkMax[]{
        new CANSparkMax(IntakeConstants.intakeMotor1ID, MotorType.kBrushless),
        new CANSparkMax(IntakeConstants.intakeMotor2ID, MotorType.kBrushless),
        new CANSparkMax(IntakeConstants.intakeMotor3ID, MotorType.kBrushless),
    };

    public boolean isReadytoIntake() {
        // This function determines whether the robot is ready to intake a note.
        return false;
    }

    public boolean isReadyToHandoff() {
        // This function determines whether the robot is ready to complete the hand off.
        return false;
    }

    public Command intakeNote(){
        return new InstantCommand(
            this::setIntakeMotorSpeeds
        );
    }

    public Command stopIntake(){
        return new InstantCommand(
            this::stopIntakeMotors
        );
    }

    private void setIntakeMotorSpeeds(){
        double motorSpeed = 0.3;
        for (CANSparkMax motor:intakeMotors){
            motor.set(motorSpeed);
        }
    }

    private void stopIntakeMotors(){
        for (CANSparkMax motor:intakeMotors){
            motor.set(0);
        }
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


