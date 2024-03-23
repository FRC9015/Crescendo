package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class autoHandoff extends ParallelCommandGroup{
    
    public autoHandoff(IntakeSubsystem intake, ShooterSubsystem shooter){
        addCommands(
            intake.runIntake(),
            shooter.runAmp());
    }
}