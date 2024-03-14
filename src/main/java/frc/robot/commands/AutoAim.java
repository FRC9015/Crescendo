package frc.robot.commands;


import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.InputManager;
import frc.robot.subsystems.LimelightInterface;

public class AutoAim extends Command{
   
    public AutoAim(){
        addRequirements(PIVOT, SWERVE);
    }
    PIDController w_pid = new PIDController(0.1, 0, 0);
    
    @Override
    public void initialize() {
        LIMELIGHT_INTERFACE.LEDsOn();
        SHOOTER.setSpeakerShooterMotorSpeeds();
        
    }
    @Override
    public void execute() {
        PIVOT.setCurrentPosition(LIMELIGHT_INTERFACE.speakerSetPoint() + LIMELIGHT_INTERFACE.offsetMultiplier());

        SWERVE.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds( //TODO Add slew rate limiting to inputs
				MathUtil.applyDeadband(-InputManager.getInstance().getDriverXYZAxes()[1], 0.15),
				MathUtil.applyDeadband(-InputManager.getInstance().getDriverXYZAxes()[0], 0.15),
				w_pid.calculate(LIMELIGHT_INTERFACE.getX()),
        SWERVE.getPose().getRotation()  
        ));
    }
    
    @Override
    public void end(boolean interrupted) {
        LIMELIGHT_INTERFACE.LEDsOff();
        SHOOTER.stopSpeakerShooterMotors();
        PIVOT.intake();        
    }
    
}
