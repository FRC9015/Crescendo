package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Constants.PivotConstants;
import frc.robot.RobotSelf.RobotSelves;

public class PivotSubsystem extends SubsystemBase {
    //makes motors
    public final CANSparkFlex pivotMotor1 = new CANSparkFlex(PivotConstants.pivotMotor1ID, CANSparkLowLevel.MotorType.kBrushless);
    public final CANSparkFlex pivotMotor2 = new CANSparkFlex(PivotConstants.pivotMotor2ID, CANSparkLowLevel.MotorType.kBrushless);
    
    //gets encoders
    public final RelativeEncoder pivotEncoder1 = pivotMotor1.getEncoder();
    public final RelativeEncoder pivotEncoder2 = pivotMotor2.getEncoder();
    
    //makes PIDs for both motors
    private final SparkPIDController pivot_PidController1 = pivotMotor1.getPIDController();
    private final SparkPIDController pivot_PidController2 = pivotMotor2.getPIDController();


    public PivotSubsystem(){
        
        //sets PID values of both controllers
        pivot_PidController1.setP(1);
        pivot_PidController1.setI(0);
        pivot_PidController1.setD(0);

        pivot_PidController2.setP(1);
        pivot_PidController2.setI(0);
        pivot_PidController2.setD(0);
        //makes encoders account for gear box
        pivotEncoder1.setPositionConversionFactor(1.0/9);
        pivotEncoder2.setPositionConversionFactor(1.0/9);
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
        pivotMotor2.set(-motorSpeed);
        
    }
    //stops pivot
    private void stopPivot(){
        pivotMotor1.stopMotor();
        pivotMotor2.stopMotor();
    }
    //moves pivot
    private void movePivotDown(){
        double motorSpeed = -0.1;
        pivotMotor1.set(motorSpeed);
        pivotMotor2.set(-motorSpeed);
    }

    //uses SparkMax PID to set the motors to a position
    private void intake(){
        //                               what position              //what measurement it uses
        pivot_PidController1.setReference(0.3, CANSparkMax.ControlType.kPosition);
        pivot_PidController2.setReference(-0.2, CANSparkMax.ControlType.kPosition);

        //sets different PID values based on preset
        pivot_PidController1.setP(1);
        pivot_PidController1.setI(0);
        pivot_PidController1.setD(0);

        pivot_PidController2.setP(1);
        pivot_PidController2.setI(0);
        pivot_PidController2.setD(0);
    }

    
    //uses SparkMax PID to set the motors to a position
    private void SubWoofer(){
        pivot_PidController1.setReference(0, CANSparkMax.ControlType.kPosition);
        pivot_PidController2.setReference(0, CANSparkMax.ControlType.kPosition);

        //sets different PID values based on preset
        pivot_PidController1.setP(0.4);
        pivot_PidController1.setI(0);
        pivot_PidController1.setD(0);


        pivot_PidController2.setP(0.4);
        pivot_PidController2.setI(0);
        pivot_PidController2.setD(0);
    }

    //uses SparkMax PID to set the motors to a position
    private void AmpPreset(){
        pivot_PidController1.setReference(0.9, CANSparkMax.ControlType.kPosition);
        pivot_PidController2.setReference(-0.9, CANSparkMax.ControlType.kPosition);
       
        //sets different PID values based on preset
        pivot_PidController1.setP(1);
        pivot_PidController1.setI(0);
        pivot_PidController1.setD(0);


        pivot_PidController2.setP(1);
        pivot_PidController2.setI(0);
        pivot_PidController2.setD(0);
    }

    //zeros encoder
    public  void zeroEncoder(){
        pivotEncoder1.setPosition(0);
        pivotEncoder2.setPosition(0);
    }

    @Override
    public void periodic(){
        //puts values on dashboard
        SmartDashboard.putNumber("pivot Position 1", pivotEncoder1.getPosition());
        SmartDashboard.putNumber("pivot Position 2", pivotEncoder2.getPosition());

        SmartDashboard.putBoolean("intake", RobotSelves.getIntakeSelf());
        SmartDashboard.putBoolean("SubWoofer", RobotSelves.getSubWooferSelf());
        SmartDashboard.putBoolean("AmpPreset", RobotSelves.getAmpPrestSelf());

        //uses robotSelf booleans to decide if to run a command
        if(RobotSelves.getIntakeSelf()){
            intake();
        }

        if(RobotSelves.getSubWooferSelf()){
            SubWoofer();
        }

        if(RobotSelves.getAmpPrestSelf()){
            AmpPreset();
            
        }
    }
}