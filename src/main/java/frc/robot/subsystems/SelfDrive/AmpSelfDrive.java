package frc.robot.subsystems.SelfDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotSelf;
import frc.robot.RobotSelf.RobotSelves;
import frc.robot.Utils.Transform2d;
import frc.robot.commands.FollowTag;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import static frc.robot.RobotContainer.driveController;
import static frc.robot.RobotContainer.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.*;


public class AmpSelfDrive extends SubsystemBase{

    //brings in other subsystems to be used
    private SwerveSubsystem drive;
    private LimelightInterface limelight;
    private CommandXboxController controller;
    private IMU imu;
   
    
    //makes the subsystems exist and usable
    public AmpSelfDrive(CommandXboxController drivecontroller, LimelightInterface limelight,SwerveSubsystem drive, IMU imu) {
        this.controller = drivecontroller;
        this.imu = imu;
        this.limelight = limelight;
        this.drive = drive;
        
    }
    Transform2d target;
    double rotationalVelocity = -driveController.getRightX();
    double xVelocity = driveController.getLeftX();
    double yVelocity = -driveController.getLeftY();
    
	@Override
    public void periodic(){
        //checks to see if the X button has been pressed
        if(controller.getHID().getBButtonPressed() && !RobotSelves.getSpeakerSelf()){//need to find how the x button is pressed in the command controller.
            //toggles selfdrive boolean
            RobotSelves.toggleAmpSelf();
            
            //puts selfdrive onto smartdashboard
            SmartDashboard.putBoolean("AmpSelfDrive", RobotSelves.getAmpSelf());
        }
        //makes seperate X, Y, Area values outside of the limelightinterface
        double x = limelight.getX();
        double y = limelight.getY();
        double area = limelight.getArea();
        //brings in the distance away the tag is from the bot
        double diagonalDistance = limelight.getDiagonalDistance();
        double floorDistance = limelight.getFloorDistance();
        
        //makes sure that selfdrive is true
        if(RobotSelves.getAmpSelf()){
                
                //checks for a tag using the limelightinterface commands
                if(limelight.tagCheck()){
                    //updates values
                    x = limelight.getX();
                    y = limelight.getY();
                    area = limelight.getArea();
                    diagonalDistance = limelight.getDiagonalDistance();
                    floorDistance = limelight.getFloorDistance();
                    // uses drive system to drive based on tag
                    swerve.runFollowTag(x,y,area,diagonalDistance,floorDistance);
                    
                    
                    
                }
            }
        }
    }

