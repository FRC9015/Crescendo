package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreAmp extends Command {
    private ShooterSubsystem Shooter;
    private PivotSubsystem Pivot;

    @Override
    public void execute(){
        Pivot.setGoal(180); //TODO Needs to be tuned this is a placeholder value
        Pivot.enable();
        new WaitCommand(0.6); //TODO needs to be tuned based on how long the pivot takes to get to goal
        Shooter.shootNoteToAmp();

    }

}