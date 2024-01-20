
package frc.robot.subsystems.SelfDrive;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotSelf;
import frc.robot.RobotSelf.RobotSelves;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SpeakerSelfDrive extends SubsystemBase{

    //brings in other subsystems to be used
    private SwerveSubsystem drive;
    private LimelightInterface limelight;
    private CommandXboxController controller;
    
    
    //makes the subsystems exist and usable
    public SpeakerSelfDrive(CommandXboxController controller, LimelightInterface limelightInterface, SwerveSubsystem swerve) {
        this.controller = controller;
        this.limelight = limelightInterface;
        this.drive = swerve;
        
    }
    @Override
    public void periodic(){
        if(controller.getHID().getYButtonPressed() && !RobotSelves.getAmpSelf()){//need to find out what the y button is in the command controller
            //toggles the speaker boolean for doing speaker self drive
            RobotSelves.toggleSpeakerSelf();
            //puts speakerSelf onto smartdashboard
            SmartDashboard.putBoolean("SpeakerSelf",RobotSelves.getSpeakerSelf());
        }

        //makes variables for the X Y Area and Distance of the limelight in SpeakerSelfDrive
        double x = limelight.getX();
        double y = limelight.getY();
        double area = limelight.getArea();
        double diagonalDistance = limelight.getDiagonalDistance();
        double floorDistance = limelight.getFloorDistance();
            
        if(RobotSelves.getAmpSelf()){
            //updates variables
            x = limelight.getX();
            y = limelight.getY();
            area = limelight.getArea();
            diagonalDistance = limelight.getDiagonalDistance();
            floorDistance = limelight.getFloorDistance();
        }
    }
}
