package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Constants.LimelightConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightInterface extends SubsystemBase{

    private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    private static boolean tag = false;
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
        //updates the X,Y,Area values
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        SmartDashboard.putNumber("SpeakerSetPoint", getSetPoint());
        SmartDashboard.putNumber("Distance", getSpeakerDistance());

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
        if(getArea() > 0.1){
            tag = true;
        }else{
            tag = false;
        }
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

        double distance = 
            (LimelightConstants.aprilTag_Height - LimelightConstants.LimelightHeight)
                / Math.tan(
                    (Math.PI / 180.0)
                        * (LimelightConstants.LimelightAngle + getTY()));
        
        return distance/Math.cos(getTX() * (Math.PI / 180));

    }

    public double getSetPoint(){
        double distance = getSpeakerDistance();

        return (78.3 * Math.exp(-0.0177*distance)) + 25.04;
    }
}