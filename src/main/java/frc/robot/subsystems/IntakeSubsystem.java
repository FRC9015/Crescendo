package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Constants.IntakeConstants;

import java.util.function.BooleanSupplier;

public class IntakeSubsystem extends SubsystemBase {
    public final DigitalOutput speakerSensor = new DigitalOutput(0);
    private final DigitalOutput handoffSensor = new DigitalOutput(1);
    private CANSparkFlex[] intakeMotors = new CANSparkFlex[]{
        new CANSparkFlex(IntakeConstants.intakeMotor1ID, MotorType.kBrushless),
    };
    private final CANSparkFlex handoffMotor = new CANSparkFlex(IntakeConstants.handoffMotorID, MotorType.kBrushless);
    RelativeEncoder handoffMotorEncoder = handoffMotor.getEncoder();

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

    public Command autoIntakeNote(){
        return new SequentialCommandGroup(
                new InstantCommand(this::setIntakeMotorSpeeds),
                new WaitCommand(1),
                new InstantCommand(this::stopIntakeMotors));
    }

    public Command outtakeNote(){
        return this.startEnd(
                this::setReverseIntakeMotorSpeeds,
                this::stopIntakeMotors
        );
    }

    public Command autoOuttakeNote(){
        return new SequentialCommandGroup(
                new InstantCommand(this::setReverseIntakeMotorSpeeds),
                new WaitCommand(3),
                new InstantCommand(this::stopIntakeMotors));
    }

    public Command stopIntake(){
        return this.runOnce(
            this::stopIntakeMotors
        );
    }


    private void setIntakeMotorSpeeds(){
        double motorSpeed = -0.8;
        for (CANSparkFlex motor : intakeMotors) {
            motor.set(motorSpeed);
        }
        handoffMotor.set(-motorSpeed);
    }
    private void setReverseIntakeMotorSpeeds(){
        double motorSpeed = 0.8;
        for(CANSparkFlex motor:intakeMotors){
            motor.set(motorSpeed);
        }
        handoffMotor.set(-motorSpeed);
    }
    private void stopIntakeMotors(){
        for (CANSparkFlex motor:intakeMotors){
            motor.stopMotor();
        }
        handoffMotor.stopMotor();
    }

    public double getHandoffMotorRPM(){
        return handoffMotorEncoder.getVelocity();
    }
    public boolean noteInPosition() {
        return speakerSensor.get();
    }

    public boolean intakeRunning(){
        return (getHandoffMotorRPM()>=0.3*6784);
    }
    public boolean getHandoffStatus(){

        return handoffSensor.get();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Note In?", speakerSensor.get());
        SmartDashboard.putBoolean("Note In Handoff?",handoffSensor.get());

    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    }


