package frc.robot.commands.Presets;


import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.PIVOT;


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