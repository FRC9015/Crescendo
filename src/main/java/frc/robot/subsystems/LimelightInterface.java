package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;


    
public class LimelightInterface extends SubsystemBase{
    
    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
    
    
    private static boolean tag = false;
    //takes the X,Y, and area values from the limelight networktable
    NetworkTableEntry tx = limelight.getEntry("tx");//Tag X value
    NetworkTableEntry ty = limelight.getEntry("ty");//Tag Y value
    NetworkTableEntry ta = limelight.getEntry("ta");//Tag Area
    

    //makes variables for the X Y and Area values of the limelight
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    //updates dashboard
    private void updateDashboard(double x, double y, double area,double diagonalDistance, double floorDistance) {
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area); 
        SmartDashboard.putNumber("Diagonal Distance",diagonalDistance);
        SmartDashboard.putNumber("Floor Distance", floorDistance);

    }

    //Used to calculate the distance from a tag
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 330.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 6.5; 

    // distance from the target to the floor
    double goalHeightInches = 44; 

    
    double angleToGoalDegrees = 0;
    double theta = 0;
    double diagonalDistance = 0;
    double height = 0;
    double floorDistance = 0;
    //updates limelight X, Y, and Area and puts them onto smartdashboard.
    @Override
    public void periodic() {
        
        System.out.println("botPose" + LimelightHelpers.getBotPose2d("limelight"));
        System.out.println("target bot pose" + LimelightTarget_Retro.);

        //updates the X,Y,Area values
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        angleToGoalDegrees = limelightMountAngleDegrees - y;
        theta = angleToGoalDegrees * (Math.PI / 180.0);
        height = limelightLensHeightInches - goalHeightInches;
            if(tagCheck()){
            
            diagonalDistance = height / Math.tan(theta);
            
            //A^2 + B^2 = C^2
            floorDistance = Math.sqrt((diagonalDistance*diagonalDistance)-(height*height));
        }
        else {
            diagonalDistance = 0;
            floorDistance = 0;
        }
        //updates smartdashboard with values
        updateDashboard(x, y, area,diagonalDistance,floorDistance);
        
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
    
    public double getDiagonalDistance(){
        return diagonalDistance;
    }
    public double getFloorDistance(){
        return floorDistance;
    }
    public double getTheta(){
        return theta;
    }
    
    public boolean tagCheck(){
        if(getArea() > 0.1){
            tag = true;
        }else{
            tag = false;
        }
        return tag;
    }

}
