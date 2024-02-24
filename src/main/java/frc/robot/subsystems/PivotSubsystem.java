package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.Constants.PivotConstants;



public class PivotSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkFlex pivotMotor1 = new CANSparkFlex(PivotConstants.pivotMotor1ID, CANSparkLowLevel.MotorType.kBrushless);
    //private final CANSparkFlex pivotMotor2 = new CANSparkFlex(PivotConstants.pivotMotor2ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder pivotEncoder = pivotMotor1.getEncoder();
    private final ArmFeedforward pivotFeedForward =
            new ArmFeedforward(
                    1,1.26,0.15,0.02);  //kSVolts, kGVolts, kVVoltSecondPerRad, kAVoltSecondSquaredPerRad

    public PivotSubsystem(){
        super(
                new ProfiledPIDController(
                        0,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                0, //kMaxVelocityRadPerSecond,
                                0 //kMaxAccelerationRadPerSecSquared)),
                        )));

        pivotEncoder.setPositionConversionFactor(1.0/9);
        pivotMotor1.setSmartCurrentLimit(40);
        //pivotMotor2.setSmartCurrentLimit(40);
        pivotMotor1.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward,180); //TODO tune
        //pivotMotor2.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,-90); //TODO tune
        // Start arm at rest in neutral position
        setGoal(0); //TODO Add Pivot Encoder Offset in Radians

    }
    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the setpoint
        double feedforward = pivotFeedForward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        pivotMotor1.setVoltage(output + feedforward);
        //pivotMotor2.setVoltage(output + feedforward);
    }

    @Override
    public double getMeasurement() {
        return pivotEncoder.getPosition() + 0;  //TODO Find the Pivot Encoder Offset
    }

    public Command zeroPivot(){
        return run(()->{
            pivotEncoder.setPosition(0);
        });
    }
    public Command raisePivot(){
        return startEnd(
                this::movePivotUp,
                this::stopPivot
        );
    }
    public Command lowerPivot(){
        return startEnd(
                this::movePivotDown,
                this::stopPivot
        );
    }
    private void movePivotUp(){
        double motorSpeed = 0.2;
        pivotMotor1.set(motorSpeed);
    }
    private void stopPivot(){
        pivotMotor1.stopMotor();
    }
    private void movePivotDown(){
        double motorSpeed = -0.2;
        pivotMotor1.set(motorSpeed);
    }
}
