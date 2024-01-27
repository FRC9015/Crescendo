package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import frc.robot.Constants.Constants.OperatorConstants;

/**
 * This handles ALL input from the controller.
 * DO NOT FUCKING TOUCH THIS CLASS WITHOUT ME!
 * - Matthew
 */
public class InputManager {

    public enum Button{
        A_Button1(1),
        B_Button2(2),
        X_Button3(3),
        Y_Button4(4),
        LB_Button5(5),
        RB_Button6(6);

        public final int buttonID;

        Button(int id){
            buttonID = id;
        }
    }

    private static InputManager Instance;
    private CommandGenericHID driveController;
    private boolean hasBeenInitialized;

    private InputManager(){
        driveController = new CommandGenericHID(OperatorConstants.kDriverControllerPort);
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
        driveController.button(button.buttonID).onTrue(command);
    }

    public Translation2d getSwerveVelocity2D(){
        return new Translation2d(driveController.getRawAxis(0), -driveController.getRawAxis(1));
    }

    public double getSwerveRotationalVelocity(){
        if (driveController.getHID().getType() == GenericHID.HIDType.kXInputGamepad){
            return -driveController.getRawAxis(4);
        } else{
            return -driveController.getRawAxis(2);
        }
    }
}
