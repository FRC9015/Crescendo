package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.FieldConstants;
import frc.robot.Constants.Constants.LimelightConstants;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;

import java.lang.reflect.Field;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightInterface extends SubsystemBase{

    private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    private static boolean tag = false;

    private double noteVelocity = 15;
    //takes the X,Y, and area values from the limelight networktable
    NetworkTableEntry tx = limelight.getEntry("tx");//Tag X value
    NetworkTableEntry ty = limelight.getEntry("ty");//Tag Y value
    NetworkTableEntry ta = limelight.getEntry("ta");//Tag Area

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("limelight");
    
    //makes variables for the X Y and Area values of the limelight
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
   
    //updates limelight X, Y, and Area and puts them onto smartd95ashboard.
    @Override
    public void periodic() {
        
        if(getArea() > 0.1){
            tag = true;
        }else{
            tag = false;
        }
        //updates the X,Y,Area values
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        SmartDashboard.putNumber("SpeakerSetPoint", getSetPoint());
        SmartDashboard.putBoolean("April Tag", tag);
        SmartDashboard.putNumber("distance", getSpeakerDistance());
        SmartDashboard.putString("Angle", getSpeakerAngle().toString());

    }

    public double getX(){
        return tx.getDouble(0.0);
    }

    public double getY(){
        return ty.getDouble(0.0);
    }

    public double getArea(){
        return ta.getDouble(0.0);
    }

    public boolean tagCheck(){
        return tag;
    }

    public void LEDsOn(){
        table.getEntry("ledMode").setNumber(3);
    }

    public void LEDsOff(){
        table.getEntry("ledMode").setNumber(1);
    }

    
    @AutoLogOutput
    public double getTY(){
        return LimelightHelpers.getLimelightNTDouble("limelight", "ty");
    }

    @AutoLogOutput
    public double getTX(){
        return LimelightHelpers.getLimelightNTDouble("limelight", "tx");
    }

    public double getSpeakerDistance(){
       var speakerPose = (RobotContainer.IsRed() ? FieldConstants.Speaker_Red_Pose: FieldConstants.Speaker_Blue_Pose);
    
        return POSE_ESTIMATOR.getEstimatedPose().getTranslation().getDistance(speakerPose); 
    }

    public Rotation2d getSpeakerAngle(){
        var speakerPose = (RobotContainer.IsRed() ? FieldConstants.Speaker_Red_Pose: FieldConstants.Speaker_Blue_Pose);

        return POSE_ESTIMATOR.getEstimatedPose().getTranslation().minus(speakerPose).getAngle();
    }

    public double getAngleToSpeaker(){
        double distance = getSpeakerDistance();

        return 56.3231 - (6.9771 * distance);
    }

    public double getSetPoint(){
        
        return (0.484411 - (0.0058831 * getTargetAngle()));
    }

    public double getTargetAngle(){

        double flightTime = getSpeakerDistance()/noteVelocity;
        double dropDistance = (9.8/2) * (flightTime*flightTime);
        double newHeight = dropDistance  + LimelightConstants.speakerGoalHeight;

        return Units.radiansToDegrees(Math.atan(Math.abs(newHeight/getSpeakerDistance())));
    }
}