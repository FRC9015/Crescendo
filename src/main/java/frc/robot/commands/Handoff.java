package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class Handoff extends ParallelCommandGroup {
//        PIVOT.setGoal(0); //TODO Make this the Encoder's offset
//        PIVOT.enable();
//        new WaitCommand(0.6); //Tune value
    public Handoff(IntakeSubsystem intake, ShooterSubsystem shooter){
        addCommands(
                intake.intakeNote(),
                shooter.ampIntake()
                
        );
    }
}