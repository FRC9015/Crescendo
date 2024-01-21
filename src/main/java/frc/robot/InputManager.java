package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Constants.OperatorConstants;

public class InputManager {

    private static InputManager Instance;
    private CommandXboxController driveController;

    private InputManager(){
        driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    }

    public static InputManager getInstance(){
        if (Instance == null){
            Instance = new InputManager();
        }

        return Instance;
    }

    public Translation2d getSwerveVelocity2D(){
        return new Translation2d(driveController.getLeftX(), -driveController.getLeftY());
    }

    public double getSwerveRotationalVelocity(){
        return -driveController.getRightX();
    }

    public void setSwervePrintOffsetButton(Command command){
        driveController.a().onTrue(command);
    }

    public void setZeroGyroButton(Command command){
        driveController.x().onTrue(command);
    }
}
