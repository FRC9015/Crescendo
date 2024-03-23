package frc.robot.Commands.Presets;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.RobotContainer.PIVOT;
import static frc.robot.RobotContainer.SHOOTER;

public class SubwooferPreset extends Command {

    public SubwooferPreset(){
        addRequirements(PIVOT);
    }
 
    @Override
    public void initialize() {
       PIVOT.SubWoofer();
    }
    @Override
    public void execute() {   
    }

    @Override
    public void end(boolean interrupted) {
        PIVOT.intake();
    }   
       

}