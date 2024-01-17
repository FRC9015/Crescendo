package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Constants.OperatorConstants;

public class InputManager {

    private static InputManager Instance;
    private CommandXboxController driveController;

    public InputManager(){
        driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    }

    public InputManager getInstance(){
        if (Instance == null){
            Instance = new InputManager();
        }

        return Instance;
    }
}
