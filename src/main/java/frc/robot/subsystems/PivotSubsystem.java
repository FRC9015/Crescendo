package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;

import frc.robot.Constants.Constants.PivotConstants;


public class PivotSubsystem extends SubsystemBase {
    
    //makes motors
    public final CANSparkFlex pivotMotor1 = new CANSparkFlex(PivotConstants.pivotMotor1ID, CANSparkLowLevel.MotorType.kBrushless);
    public final CANSparkFlex pivotMotor2 = new CANSparkFlex(PivotConstants.pivotMotor2ID, CANSparkLowLevel.MotorType.kBrushless);

    //gets encoders
    public final RelativeEncoder pivotEncoder = pivotMotor1.getEncoder();//change later


    //makes PID for motors
    private final SparkPIDController pivotPIDController = pivotMotor1.getPIDController();

    //motion profiling
    private final TrapezoidProfile pivot1Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    private final TrapezoidProfile pivot2Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    TrapezoidProfile.State motor1Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor1Goal = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Goal = new TrapezoidProfile.State();

    private double currentPosition = 0;

    public PivotSubsystem(){

        //sets PID values of both controllers
        pivotPIDController.setP(2);
        pivotPIDController.setI(0);
        pivotPIDController.setD(0);
        pivotPIDController.setOutputRange(-1,1.45);
        pivotPIDController.setFF(0.00015);


        pivotMotor2.follow(pivotMotor1,true);
        //makes encoder account for gear box/Chain
        pivotEncoder.setPositionConversionFactor(1.0/15);
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

    public Command autoAutoAim(){
        return this.runOnce(this::autoAim);
    }
    public Command movePivotToIntake(){
        return this.runOnce(this::intake);
    }

    public Command movePivotToSubWoofer(){
        return this.runOnce(this::SubWoofer);
    }
    //moves pivot up
    private void movePivotUp(){
        currentPosition += 0.05;
    }
    //stops pivot
    private void stopPivot(){
        pivotMotor1.stopMotor();
        pivotMotor2.stopMotor();
    }
    //moves pivot
    private void movePivotDown(){
        currentPosition -= 0.05;
    }

    //uses SparkMax PID to set the motors to a position
    public void intake(){
        motor1Goal = new TrapezoidProfile.State(0.5,0.5);
        motor2Goal = new TrapezoidProfile.State(-0.5,0.5);

        pivotPIDController.setP(2);
        currentPosition = 0.24;
    }

    //uses SparkMax PID to set the motors to a position
    public void SubWoofer(){
        pivotPIDController.setP(0.4);
        currentPosition = 0;
        
    }

    //uses SparkMax PID to set the motors to a position
    public void AmpPreset(){
        pivotPIDController.setP(1.5);
        currentPosition = 1.3;
        
    }

    public void passNotePreset(){
        pivotPIDController.setP(2);
        currentPosition = 0.48;
    }

    public void setCurrentPosition(double SetPoint){
        pivotPIDController.setP(2);
        currentPosition = SetPoint;
    }

    public void autoAim(){
        setCurrentPosition(LIMELIGHT_INTERFACE.getSetPoint());
    }

    @Override
    public void periodic(){
        //puts values on dashboard
        SmartDashboard.putNumber("pivot Position", pivotEncoder.getPosition());
        
        double kDt = 0.02;
        motor1Setpoint = pivot1Profile.calculate(kDt,motor1Setpoint,motor1Goal);
        motor2Setpoint = pivot2Profile.calculate(kDt,motor2Setpoint,motor2Goal);

        pivotPIDController.setReference(currentPosition, CANSparkFlex.ControlType.kPosition);
        
    }
}