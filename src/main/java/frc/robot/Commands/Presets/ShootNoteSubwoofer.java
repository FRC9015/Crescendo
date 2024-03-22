package frc.robot.Commands.Presets;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.RobotContainer.PIVOT;
import static frc.robot.RobotContainer.SHOOTER;

public class ShootNoteSubwoofer extends SequentialCommandGroup {
    public ShootNoteSubwoofer(){
        addRequirements(PIVOT,SHOOTER);
        addCommands(PIVOT.movePivotToSubWoofer(),
                SHOOTER.autoShootNoteToSpeaker(),
                new WaitCommand(1),
                PIVOT.movePivotToIntake()
        );
    }
}