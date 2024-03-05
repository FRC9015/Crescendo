package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Constants.PivotConstants;
import frc.robot.RobotSelf.RobotSelves;

public class PivotSubsystem extends SubsystemBase {
    private final double kDt = 0.02;

    //makes motors
    public final CANSparkFlex pivotMotor1 = new CANSparkFlex(PivotConstants.pivotMotor1ID, CANSparkLowLevel.MotorType.kBrushless);
    public final CANSparkFlex pivotMotor2 = new CANSparkFlex(PivotConstants.pivotMotor2ID, CANSparkLowLevel.MotorType.kBrushless);
    
    //gets encoders
    public final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);//change later

    //makes PID for motors
    private final SparkPIDController pivotPIDController = pivotMotor1.getPIDController();

    //motion profiling
    private final TrapezoidProfile pivot1Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    private final TrapezoidProfile pivot2Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    TrapezoidProfile.State motor1Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor1Goal = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Goal = new TrapezoidProfile.State();

    public PivotSubsystem(){

        //sets PID values of both controllers
        pivotPIDController.setP(2);
        pivotPIDController.setI(0);
        pivotPIDController.setD(0);
        pivotPIDController.setOutputRange(-1,1);
        pivotPIDController.setFF(0.00015);


        pivotMotor2.follow(pivotMotor1,true);
        //makes encoder account for gear box/Chain
        pivotEncoder.setDistancePerRotation(1/3);
        
    }

    //raises the pivot
    public Command raisePivot(){
        return startEnd(
                this::movePivotUp,
                this::stopPivot
        );
    }
    //lowers the pivot
    public Command lowerPivot(){
        return startEnd(
                this::movePivotDown,
                this::stopPivot
        );
    }

    //moves pivot up
    private void movePivotUp(){
        double motorSpeed = 0.1;
        pivotMotor1.set(motorSpeed);
    }
    //stops pivot
    private void stopPivot(){
        pivotMotor1.stopMotor();
        pivotMotor2.stopMotor();
    }
    //moves pivot
    private void movePivotDown(){
        double motorSpeed = -0.75;
        pivotMotor1.set(motorSpeed);
    }

    //uses SparkMax PID to set the motors to a position
    public void intake(){
        motor1Goal = new TrapezoidProfile.State(0.5,0.5);
        motor2Goal = new TrapezoidProfile.State(-0.5,0.5);

        //                               what position              //what measurement it uses
        pivotPIDController.setReference(0.1, CANSparkFlex.ControlType.kPosition);

    }

    //uses SparkMax PID to set the motors to a position
    private void SubWoofer(){
        pivotPIDController.setP(1.5);

        pivotPIDController.setReference(0, CANSparkFlex.ControlType.kPosition);
    }

    //uses SparkMax PID to set the motors to a position
    public void AmpPreset(){
        pivotPIDController.setReference(0.4, CANSparkFlex.ControlType.kPosition);
    }

    @Override
    public void periodic(){
        //puts values on dashboard
        SmartDashboard.putNumber("pivot Position 1", pivotEncoder.getAbsolutePosition());

        SmartDashboard.putBoolean("intake", RobotSelves.getIntakeSelf());
        SmartDashboard.putBoolean("SubWoofer", RobotSelves.getSubWooferSelf());
        SmartDashboard.putBoolean("AmpPreset", RobotSelves.getAmpPrestSelf());

        motor1Setpoint = pivot1Profile.calculate(kDt,motor1Setpoint,motor1Goal);
        motor2Setpoint = pivot2Profile.calculate(kDt,motor2Setpoint,motor2Goal);
        //uses robotSelf booleans to decide if to run a command

        if(RobotSelves.getSubWooferSelf()){
            SubWoofer();
        }

        if(RobotSelves.getAmpPrestSelf()){
            AmpPreset();
            
        }

        if (RobotSelves.getIntakeSelf()){
            intake();
        }

        else{
            intake();
        }

    }
}