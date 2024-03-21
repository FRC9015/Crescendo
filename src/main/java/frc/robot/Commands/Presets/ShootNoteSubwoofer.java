package frc.robot.Commands.Presets;


import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class ShootNoteSubwoofer extends Command{
    public ShootNoteSubwoofer(){
        addRequirements(PIVOT,SHOOTER,INTAKE);
    }
 
    
    @Override
    public void initialize() {
       PIVOT.SubWoofer();
       SHOOTER.autoShootNoteToSpeaker();
    }
    @Override
    public void execute() {
        
        
    }
    
    @Override
    public void end(boolean interrupted) {
        PIVOT.intake();
    }
}
