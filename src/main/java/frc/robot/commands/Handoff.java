package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Handoff extends ParallelCommandGroup {
    public Handoff(IntakeSubsystem intake, ShooterSubsystem shooter){
        addCommands(
                intake.intakeNote(),
                shooter.ampIntake()
        );
    }
}