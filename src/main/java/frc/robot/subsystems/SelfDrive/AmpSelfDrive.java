package frc.robot.subsystems.SelfDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotSelf;
import frc.robot.RobotSelf.RobotSelves;
import frc.robot.Utils.Transform2d;
import frc.robot.commands.FollowTag;

import frc.robot.subsystems.LimelightInterface;

import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Pigeon;


import static frc.robot.RobotContainer.SWERVE;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class AmpSelfDrive extends SubsystemBase{

    //brings in other subsystems to be used
    private SwerveSubsystem drive;
    private LimelightInterface limelight;
    private CommandGenericHID controller;
    private Pigeon imu;
   
    
    //makes the subsystems exist and usable
    public AmpSelfDrive(CommandGenericHID controller, LimelightInterface limelight,SwerveSubsystem drive, Pigeon imu) {
        this.controller = controller;
        this.imu = imu;
        this.limelight = limelight;
        this.drive = drive;
        
    }
 
    
	@Override
    public void periodic(){
        //checks to see if the X button has been pressed
        if(controller.getHID().getRawButton(2) && !RobotSelves.getSpeakerSelf()){//need to find how the x button is pressed in the command controller.
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
                
                    
                    
                    
                }
            }
        }
    }

