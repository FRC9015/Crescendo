package frc.robot.Commands;

import static frc.robot.RobotContainer.INTAKE;
import static frc.robot.RobotContainer.SHOOTER;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class autoHandoff extends SequentialCommandGroup{
    
    public autoHandoff(){
        addRequirements(INTAKE,SHOOTER);
        addCommands(
                new Handoff(INTAKE, SHOOTER),
                new WaitCommand(0.5),
                SHOOTER.stopAmp(),
                INTAKE.stopIntake()
        );
    }
}