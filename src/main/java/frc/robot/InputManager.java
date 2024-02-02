package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.Constants.OperatorConstants;

/**
 * This handles ALL input from the controller.
 * Supports Joysticks and Xbox Controllers.
 * DON'T TOUCH THIS CLASS WITHOUT ME!
 * - Matthew
 */
public class InputManager {

    /**
     * This struct is used to handle Button Mapping on the controller.
     */
    public static class ButtonMap{
        public Button button;
        public Command command;
        public boolean whileHeld;

        /**
         * Creates the ButtonMap
         * @param button The button on the controller as a {@link Button}
         * @param command The command mapped to the button.
         * @param whileHeld Will the command run when the button is held. If false will only work on press.
         */
        public ButtonMap(Button button, Command command, boolean whileHeld){
            this.button = button;
            this.command = command;
            this.whileHeld = whileHeld;
        }
    }

    public enum Button{
        A_Button1(1),
        B_Button2(2),
        X_Button3(3),
        Y_Button4(4),
        LB_Button5(5),
        RB_Button6(6),
        LT_Button7(7),
        RT_Button8(8);

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
     * You can pass in as many {@link ButtonMap}s as you want and any button not inputted
     * will be mapped to nothing and won't do anything. This works for both press and hold
     * commands on the controller.
     * @param controls The controls you want to map onto the controller as {@link ButtonMap}s.
     */
    public void init(ButtonMap... controls){
        if (hasBeenInitialized){
            System.err.println("Controls have already been initialized!");
            return;
        }

        for (ButtonMap buttonMap : controls){
            if (buttonMap.whileHeld){
                setButtonCommandHold(buttonMap.button, buttonMap.command);
            }else {
                setButtonCommandPressed(buttonMap.button, buttonMap.command);
            }
        }

        hasBeenInitialized = true;
    }
// TODO Operator Controller
    private void setButtonCommandPressed(Button button, Command command){
        // TODO Check if this code works on real robot
        if (driveController.getHID().getType() != GenericHID.HIDType.kXInputGamepad){
            driveController.button(button.buttonID).onTrue(command);
            return;
        }

        float pressThreshold = 0.5f;
        if (button == Button.LT_Button7){
            driveController.axisGreaterThan(2, pressThreshold).onTrue(command);
        }else if (button == Button.RT_Button8){
            driveController.axisGreaterThan(3, pressThreshold).onTrue(command);
        }else {
            driveController.button(button.buttonID).onTrue(command);
        }
    }

    private void setButtonCommandHold(Button button, Command command){
        if (driveController.getHID().getType() != GenericHID.HIDType.kXInputGamepad){
            driveController.button(button.buttonID).whileTrue(command);
            return;
        }

        float pressThreshold = 0.5f;
        if (button == Button.LT_Button7){
            driveController.axisGreaterThan(2, pressThreshold).whileTrue(command);
        }else if (button == Button.RT_Button8){
            driveController.axisGreaterThan(3, pressThreshold).whileTrue(command);
        }else {
            driveController.button(button.buttonID).whileTrue(command);
        }
    }


    public Translation2d getControllerXYAxes(){
        return new Translation2d(driveController.getRawAxis(0), -driveController.getRawAxis(1));
    }

    public double getControllerRotationalAxis(){
        if (driveController.getHID().getType() == GenericHID.HIDType.kXInputGamepad){
            return -driveController.getRawAxis(4);
        } else{
            return -driveController.getRawAxis(2);
        }
    }
    
}
