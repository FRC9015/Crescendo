package frc.robot.Commands;


import static frc.robot.RobotContainer.PIVOT;
import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;
import static frc.robot.RobotContainer.SHOOTER;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.FieldConstants;


public class AutoAim extends Command{
   
    public AutoAim(){
        addRequirements(PIVOT,LIMELIGHT_INTERFACE,SHOOTER);
    }
    PIDController w_pid = new PIDController(0.1, 0, 0);
    
    @Override
    public void initialize() {
        LIMELIGHT_INTERFACE.LEDsOn();

        
    
    }
    @Override
    public void execute() {
        PIVOT.setCurrentPosition(LIMELIGHT_INTERFACE.getSetPoint());
        SHOOTER.setSpeakerShooterMotorSpeeds();
        Logger.recordOutput("AutoAim/SpeakerPose/Blue", FieldConstants.Speaker_Blue_Pose);
    }
    
    @Override
    public void end(boolean interrupted) {
        LIMELIGHT_INTERFACE.LEDsOff();
        SHOOTER.setIdleShooterSpeeds();
        PIVOT.intake();        
    }
    
}
