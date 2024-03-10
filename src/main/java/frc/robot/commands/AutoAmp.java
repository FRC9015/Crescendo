package frc.robot.commands;


import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.InputManager;

public class AutoAmp extends Command{
    public AutoAmp(){
        addRequirements(PIVOT, SWERVE);
    }

    PIDController Y_pid = new PIDController(0.1, 0, 0);
    PIDController X_pid = new PIDController(0.1, 0, 0);
    PIDController Z_pid = new PIDController(0.1, 0, 0);
    @Override
    public void initialize() {
        LIMELIGHT_INTERFACE.LEDsOn();
        PIVOT.AmpPreset();
    }

    @Override
    public void execute() {
        
        SWERVE.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds( //TODO Add slew rate limiting to inputs
			X_pid.calculate(0),
            Y_pid.calculate(LIMELIGHT_INTERFACE.getY()),
			Z_pid.calculate(LIMELIGHT_INTERFACE.getX()),
            SWERVE.getPose().getRotation()  
            ));
        
    }

    @Override
    public void end(boolean interrupted) {
        LIMELIGHT_INTERFACE.LEDsOff();
        PIVOT.intake();
    }
}
