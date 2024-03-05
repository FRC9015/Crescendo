package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotSelf;
import frc.robot.subsystems.ShooterSubsystem;

public class SubWoofer extends ParallelCommandGroup {
    public SubWoofer(ShooterSubsystem shooter){
        addCommands(
                new InstantCommand(RobotSelf.RobotSelves::toggleSubWooferSelf),
                shooter.shootNoteToSpeaker()

        );
    }
}