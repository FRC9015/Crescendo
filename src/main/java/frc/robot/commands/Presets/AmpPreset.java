package frc.robot.commands.Presets;


import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class AmpPreset extends Command{
    public AmpPreset(){
        addRequirements(PIVOT);
    }
 
    
    @Override
    public void initialize() {
       PIVOT.AmpPreset();
    }
    @Override
    public void execute() {   
    }

    @Override
    public void end(boolean interrupted) {
        PIVOT.intake();
    }
    
}
