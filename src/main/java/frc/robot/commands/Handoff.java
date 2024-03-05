package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class Handoff extends ParallelCommandGroup {
    public Handoff(IntakeSubsystem intake, ShooterSubsystem shooter, PivotSubsystem pivot){
        addCommands(
                new InstantCommand(pivot::intake),
                intake.intakeNote(),
                shooter.ampIntake()
        );
    }
}