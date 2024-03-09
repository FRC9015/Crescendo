package frc.robot.commands;


import static frc.robot.RobotContainer.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.LimelightInterface;

public class AutoAim extends Command{
    public AutoAim(){
        addRequirements(PIVOT, SHOOTER);
    }
    @Override
    public void initialize() {
        LIMELIGHT_INTERFACE.LEDsOn();
    }
    @Override
    public void execute() {
        
    }
    
    // @Override
    // public boolean isFinished(){
    //     return false;
    // }
    
}
