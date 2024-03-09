package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotSelf.RobotSelves;

public class SpeakerSelfDrive extends SubsystemBase{

    //brings in other subsystems to be used

    private LimelightInterface limelight;

    private ShooterSubsystem Shooter;
    private PivotSubsystem Pivot;

    public SpeakerSelfDrive(LimelightInterface limelightInterface, PivotSubsystem Pivot,ShooterSubsystem Shooter) {

        this.Pivot = Pivot;
        this.limelight = limelightInterface;
        this.Shooter = Shooter;

   
    }

    double offsetMultiplier = 0;
    double multiplier = 1500;

    @Override
    public void periodic(){

        //theta angle from tag
        double theta = limelight.getTheta();
        double diagonalDistance = limelight.getDiagonalDistance();
        double floorDistance = limelight.getFloorDistance();

        offsetMultiplier = diagonalDistance / multiplier;

        if(diagonalDistance <= 50){
            multiplier = 2100;
        }else{
            multiplier = 1500;
        }

        if(diagonalDistance >= 70){
            multiplier = 1700;
        }else{
            multiplier = 1500;
        }
        
        SmartDashboard.putNumber("offset", offsetMultiplier);
        
        //where the pivot should move to

        double speakerSetPoint = (theta/(Math.PI*2));
        

        //adds things to dashboard
        SmartDashboard.putBoolean("SpeakerSelf", RobotSelves.getSpeakerSelf());
     
        //checks if speakerSelf is on
        if(RobotSelves.getSpeakerSelf()){
            limelight.LEDsOn();
            //checks if there is a tag
          if(limelight.tagCheck()){
                Pivot.setCurrentPosition(speakerSetPoint + offsetMultiplier);
                Shooter.setSpeakerShooterMotorSpeeds();

           }else{
            Shooter.stopSpeakerShooterMotors();
           }
        }else{
            limelight.LEDsOff();
        }

    }
    
    

    //resets the position of the pivot
    private void resetPosition(){
        Pivot.intake();
        //Shooter.stopSpeakerShooterMotors();
    }
}