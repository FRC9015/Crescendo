package frc.robot.Commands;


import static frc.robot.RobotContainer.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoAim extends Command{
   
    public AutoAim(){
        addRequirements(PIVOT);
    }
    PIDController w_pid = new PIDController(0.1, 0, 0);
    
    @Override
    public void initialize() {
        LIMELIGHT_INTERFACE.LEDsOn();
        SHOOTER.setSpeakerShooterMotorSpeeds();
        
    
    }
    @Override
    public void execute() {
        PIVOT.setCurrentPosition(LIMELIGHT_INTERFACE.getSetPoint());
        
    }
    
    @Override
    public void end(boolean interrupted) {
        LIMELIGHT_INTERFACE.LEDsOff();
        SHOOTER.stopSpeakerShooterMotors();
        PIVOT.intake();        
    }
    
}
