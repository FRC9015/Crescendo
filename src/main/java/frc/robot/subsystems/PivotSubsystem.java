package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.Constants.PivotConstants;


public class PivotSubsystem extends SubsystemBase {
    
    //makes motors
    public final SparkFlex pivotMotor1 = new SparkFlex(PivotConstants.pivotMotor1ID, SparkLowLevel.MotorType.kBrushless);
    public final SparkFlex pivotMotor2 = new SparkFlex(PivotConstants.pivotMotor2ID, SparkLowLevel.MotorType.kBrushless);

    //gets encoders
    //public final RelativeEncoder pivotEncoder = pivotMotor1.getEncoder();
    //makes PID for motors
   // private final SparkClosedLoopController pivotConfig.closedLoop = pivotMotor1.getClosedLoopController();

    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    //motion profiling
    private final TrapezoidProfile pivot1Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    private final TrapezoidProfile pivot2Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    TrapezoidProfile.State motor1point = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2point = new TrapezoidProfile.State();
    TrapezoidProfile.State motor1Goal = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Goal = new TrapezoidProfile.State();


    private double currentPosition = 0;
 

    public PivotSubsystem(){

        //sets PID values of both controllers
        pivotConfig.closedLoop.pid(2,0,0);
        pivotConfig.closedLoop.outputRange(-1,1.45);
        pivotConfig.closedLoop.velocityFF(0.00015);


        pivotMotor2.follow(pivotMotor1, true);
        //makes encoder account for gear box/Chain
        pivotConfig.encoder.positionConversionFactor(1.0/15);


    }

    public Command raisePivot(){
        return run(
                this::movePivotUp);
    }
    
    public Command lowerPivot(){
        return run(
                this::movePivotDown);
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

    public Command printPivotAngle(){
        return new InstantCommand(() -> System.out.println("Current Pivot Position: " + currentPosition + " Distance to Speaker: " + LIMELIGHT_INTERFACE.getSpeakerDistance()));
    }
    //moves pivot up
    private void movePivotUp(){
        currentPosition += 0.005;
    }
    //stops pivot
    private void stopPivot(){
        pivotMotor1.stopMotor();
        pivotMotor2.stopMotor();
    }
    //moves pivot
    private void movePivotDown(){
        currentPosition -= 0.005;
    }

    //uses SparkMax PID to set the motors to a position
    public void intake(){
        motor1Goal = new TrapezoidProfile.State(0.5,0.5);
        motor2Goal = new TrapezoidProfile.State(-0.5,0.5);

        pivotConfig.closedLoop.p(2);
        pivotConfig.closedLoop.i(0.0);
        currentPosition = 0.24;
    }

    //uses SparkMax PID to set the motors to a position
    public void SubWoofer(){
        pivotConfig.closedLoop.p(0.4);
        pivotConfig.closedLoop.i(0.0);
        currentPosition = 0;
        
    }

    //uses SparkMax PID to set the motors to a position
    public void AmpPreset(){
        pivotConfig.closedLoop.p(1.5);
        pivotConfig.closedLoop.i(0.0);
        currentPosition = 1.3;
        
    }

    public void passNotePreset(){
        pivotConfig.closedLoop.p(2);
        pivotConfig.closedLoop.i(0.0);
        currentPosition = 0.48;
    }

    public void setCurrentPosition(double point){
        pivotConfig.closedLoop.p(7);
        pivotConfig.closedLoop.i(0.0004);
        currentPosition = MathUtil.clamp(point, 0, 1.3);
    }

    public void autoAim(){
      
        setCurrentPosition(LIMELIGHT_INTERFACE.getpoint());
        
    }

     

    @Override
    public void periodic(){
        //puts values on dashboard
        SmartDashboard.putNumber("pivot Position", pivotMotor1.configAccessor.encoder.getPosition());
        Logger.recordOutput("Pivot/Error", pivotEncoder.getPosition()-currentPosition);
        double kDt = 0.02;
        motor1point = pivot1Profile.calculate(kDt,motor1point,motor1Goal);
        motor2point = pivot2Profile.calculate(kDt,motor2point,motor2Goal);

        pivotConfig.closedLoop.setSetpoint(currentPosition, SparkFlex.ControlType.kPosition);
        
    }
}
