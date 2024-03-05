package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.RobotContainer.PIVOT;
import static frc.robot.RobotContainer.SHOOTER;

public class ScoreAmp extends SequentialCommandGroup {
    public ScoreAmp(ShooterSubsystem shooter, PivotSubsystem pivot){
        addCommands(
            new InstantCommand(pivot::AmpPreset),
            shooter.shootNoteToAmp()

        );
    }
}