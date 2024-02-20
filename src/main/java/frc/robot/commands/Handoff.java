package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static frc.robot.RobotContainer.INTAKE;
import static frc.robot.RobotContainer.PIVOT;
import static frc.robot.RobotContainer.SHOOTER;



public class Handoff extends Command {
    @Override
    public void execute(){
        PIVOT.setGoal(0); //TODO Make this the Encoder's offset
        PIVOT.enable();
        new WaitCommand(0.6); //Tune value
        INTAKE.intakeNote();
        SHOOTER.ampIntake();

    }

}