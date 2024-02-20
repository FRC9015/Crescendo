package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Handoff extends Command {
    private IntakeSubsystem Intake;
    private ShooterSubsystem Shooter;
    private PivotSubsystem Pivot;

    @Override
    public void execute(){
        Intake.intakeNote();
        Pivot.setGoal(0); //TODO Make this the Encoder's offset
        Pivot.enable();
        Shooter.ampIntake();
    }

}