package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


    
public class LimelightInterface extends SubsystemBase{
    
    private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");


    //takes the X,Y, and area values from the limelight networktable
    NetworkTableEntry tx = limelight.getEntry("tx");//Tag X value
    NetworkTableEntry ty = limelight.getEntry("ty");//Tag Y value
    NetworkTableEntry ta = limelight.getEntry("ta");//Tag Area

    //makes variables for the X Y and Area values of the limelight
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    //updates dashboard
    private void updateDashboard(double x, double y, double area,double distance) {
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area); 
        SmartDashboard.putNumber("TagDistance",distance);
    }

    //Used to calculate the distance from a tag
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 31.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 6.5; 

    // distance from the target to the floor
    double goalHeightInches = 35; 

    
    double angleToGoalDegrees = 0;
    double angleToGoalRadians = 0;
    double distance = 0;
    //updates limelight X, Y, and Area and puts them onto smartdashboard.
    @Override
    public void periodic() {
        //updates the X,Y,Area values
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        angleToGoalDegrees = limelightMountAngleDegrees + y;
            angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        if(tagCheck()){
            
            distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        }
        else {
            distance = 0;
        }
        //updates smartdashboard with values
        updateDashboard(x, y, area,distance);
        
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
    
    public double getDistance(){
        return distance;
    }
    
    public boolean tagCheck(){
        return getArea() > 0.1;
    }
}
