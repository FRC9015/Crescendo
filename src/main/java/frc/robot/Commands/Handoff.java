package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Handoff extends ParallelCommandGroup {

   
    public Handoff(IntakeSubsystem intake, AmpSubsystem amp){
        addCommands(
                intake.intakeNote(),
                amp.ampIntake()
        );
        
   
    }
}