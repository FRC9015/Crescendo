package frc.robot.Commands.Presets;


import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.PIVOT;
import static frc.robot.RobotContainer.SHOOTER;


public class RingTossPreset extends Command {

    public RingTossPreset(){
        addRequirements(PIVOT,SHOOTER);
    }

    @Override
    public void initialize() {
        PIVOT.RingToss();

    }
    @Override
    public void execute() {
        SHOOTER.setSpeakerShooterMotorSpeedsRingToss();
    }

    @Override
    public void end(boolean interrupted) {
        PIVOT.intake();
        SHOOTER.setIdleShooterSpeeds();
    }


}