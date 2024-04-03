package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.PIVOT;

public class PassNotePreset extends Command{
    public PassNotePreset(){
        addRequirements(PIVOT);
        addRequirements(SHOOTER);
    }
 
    @Override
    public void initialize() {
       PIVOT.passNotePreset();
       SHOOTER.setSpeakerShooterMotorSpeedsSubWoofer();
    }
    @Override
    public void execute() {   
    }

    @Override
    public void end(boolean interrupted) {
        PIVOT.intake();
        SHOOTER.setIdleShooterSpeeds();
    }   
}
