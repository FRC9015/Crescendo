
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

    @Override
    public void periodic(){

        //theta angle from tag
        double theta = 20;

        //where the pivot should move to

    //    double speakerSetPoint = ((Math.PI*2) - theta) /(Math.PI*2);
            double speakerSetPoint = (theta /360) + 0.1;

        //adds things to dashboard
        SmartDashboard.putBoolean("SpeakerSelf", RobotSelves.getSpeakerSelf());
        SmartDashboard.putNumber("Pivot Position", Pivot.pivotEncoder.getPosition());
        SmartDashboard.putNumber("Speaker SetPoint",speakerSetPoint);

        //checks if speakerSelf is on
        if(RobotSelves.getSpeakerSelf()){

            //checks if there is a tag
          if(limelight.tagCheck()){

                Pivot.setGoal(1);
                Pivot.enable();

                if(Pivot.pivotEncoder.getPosition() >= speakerSetPoint){
                    Shooter.setSpeakerShooterMotorSpeeds();
                }


                if(Pivot.pivotEncoder.getPosition() >= speakerSetPoint){
                    Shooter.setSpeakerShooterMotorSpeeds();
                }

           }
        }
    }
    

    //resets the position of the pivot
    private void resetPosition(){
        Pivot.setGoal(0);
        Pivot.enable();
        Shooter.stopSpeakerShooterMotors();
    }
}

