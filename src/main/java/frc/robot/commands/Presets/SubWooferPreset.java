package frc.robot.commands.Presets;


import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class SubWooferPreset extends Command{
    public SubWooferPreset(){
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
