package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Constants.OperatorConstants;

/**
 * This handles ALL input from the controller.
 * DO NOT FUCKING TOUCH THIS CLASS WITHOUT ME!
 * - Matthew
 */
public class InputManager {

    public enum Button{
        A,
        B,
        X,
        Y,
        LB,
        RB,
        LT,
        RT
    }

    private static InputManager Instance;
    private final CommandXboxController driveController;
    private boolean hasBeenInitialized;

    private InputManager(){
        driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    }

    public static InputManager getInstance(){
        if (Instance == null){
            Instance = new InputManager();
        }

        return Instance;
    }

    /**
     * Initializes the Controls for the buttons on the controller.
     * You can pass in as many buttons and commands as you need and if a button isn't stated, it's mapped to nothing.
     * Make sure that every {@link Button} you put has a {@link Command} to go with it and that the Button goes first.
     * @param controls The Controls you would like to initialize. Must follow the pattern
     *                 ({@link Button}, {@link Command}...)
     */
    public void init(Object... controls){
        if (hasBeenInitialized){
            System.err.println("Controls have already been initialized!");
            return;
        }

        for (int i = 0; i < controls.length; i++) {
            if ((i % 2 == 0 && !(controls[i] instanceof Button)) || (i % 2 == 1 && !(controls[i] instanceof Command))){
                System.err.println("Parameters must follow pattern (Button, Command, Button, Command,...)!");
            }
        }

        if (controls.length % 2 != 0) System.err.println("Every Button must have a Command");

        for (int i = 0; i < controls.length; i += 2) {
            Button button = (Button) controls[i];
            Command command = (Command) controls[i + 1];

            setButtonCommand(button, command);
        }

        hasBeenInitialized = true;
    }

    private void setButtonCommand(Button button, Command command){
        switch (button){
            case A -> driveController.a().onTrue(command);
            case B -> driveController.b().onTrue(command);
            case X -> driveController.x().onTrue(command);
            case Y -> driveController.y().onTrue(command);
            case LB -> driveController.leftBumper().onTrue(command);
            case RB -> driveController.rightBumper().onTrue(command);
            case LT -> driveController.leftTrigger().onTrue(command);
            case RT -> driveController.rightTrigger().onTrue(command);
        }
    }

    public Translation2d getSwerveVelocity2D(){
        return new Translation2d(driveController.getLeftX(), -driveController.getLeftY());
    }

    public double getSwerveRotationalVelocity(){
        return -driveController.getRightX();
    }
}
