package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Constants.InputConstants;
import frc.robot.Constants.Constants.OperatorConstants;

/**
 * This handles ALL input from the controller.
 * Supports Joysticks and Xbox Controllers.
 * DON'T TOUCH THIS CLASS WITHOUT ME!
 * - Matthew
 */
public class InputManager {
    /**
     * The enum that stores the ids of each type of button.
     * Works for both Xbox Controllers and Joysticks.
     */
    public enum Button{
        A_Button1(1),
        B_Button2(2),
        X_Button3(3),
        Y_Button4(4),
        LB_Button5(5),
        RB_Button6(6),
        LT_Button7(7),
        RT_Button8(8),
        Shoot_Button(11),
        Intake_Button(12),
        Aim_Lock_Button(13),
        Pivot_Shooter_Button(14),
        Pivot_Amp_Button(15);

        public final int buttonID;

        Button(int id){
            buttonID = id;
        }
    }

    private static InputManager Instance;
    private CommandGenericHID driveController;
    private CommandGenericHID operatorController;

    private double pressThreshold = InputConstants.defaultTriggerPressThreshold;

    private InputManager(){
        driveController = new CommandGenericHID(OperatorConstants.driverControllerPort);
        operatorController = new CommandGenericHID(OperatorConstants.operatorControllerPort);

        Preferences.initDouble(InputConstants.triggerPressThresholdKey, pressThreshold);
    }

    /**
     * @return the singleton instance of InputManager. If it doesn't exist it creates one and returns that.
     */
    public static InputManager getInstance(){
        if (Instance == null){
            Instance = new InputManager();
        }

        return Instance;
    }

    /**
     * Gets the button from the drive controller.
     * If it is an Xbox controller, and you want the triggers,
     * it will give you when that is greater than some press threshold.
     * Otherwise, will return button with id in name.
     * @param button The button you want to get from the drive controller as a {@link Button}
     * @return Button from controller
     */
    public Trigger getDriverButton(Button button){
        if (driveController.getHID().getType() != GenericHID.HIDType.kXInputGamepad) {
            return driveController.button(button.buttonID);
        }

        return switch (button) {
            case LT_Button7 -> driveController.axisGreaterThan(2, Preferences.getDouble(InputConstants.triggerPressThresholdKey, pressThreshold));
            case RT_Button8 -> driveController.axisGreaterThan(3, Preferences.getDouble(InputConstants.triggerPressThresholdKey, pressThreshold));
            default -> driveController.button(button.buttonID);
        };
    }

    /**
     * Gets the button from the operator controller.
     * If it is an Xbox controller and you want the triggers,
     * it will give you when that is greater than some press threshold.
     * Otherwise, will return button with id in name.
     * @param button The button you want to get from the operator controller as a {@link Button}
     * @return Button from controller
     */
    public Trigger getOperatorButton(Button button){
        if (operatorController.getHID().getType() != GenericHID.HIDType.kXInputGamepad) {
            return operatorController.button(button.buttonID);
        }

        return switch (button) {
            case LT_Button7 -> operatorController.axisGreaterThan(2, Preferences.getDouble(InputConstants.triggerPressThresholdKey, pressThreshold));
            case RT_Button8 -> operatorController.axisGreaterThan(3, Preferences.getDouble(InputConstants.triggerPressThresholdKey, pressThreshold));
            default -> operatorController.button(button.buttonID);
        };
    }

    /**
     * Gets the X, Y, and Z axes of the drive controller
     * where the Z axis is the rotational axis.
     * @return The values of the X, Y, and Z axes as a double array.
     */
    public double[] getDriverXYZAxes(){
        if (driveController.getHID().getType() != GenericHID.HIDType.kXInputGamepad) {
            return new double[]{
                    driveController.getRawAxis(0),
                    driveController.getRawAxis(1),
                    driveController.getRawAxis(4)};
        }

        return new double[]{
                driveController.getRawAxis(0),
                driveController.getRawAxis(1),
                driveController.getRawAxis(4)};
    }

    /**
     * Gets the X, Y, and Z axes of the operator controller
     * where the Z axis is the rotational axis.
     * @return The values of the X, Y, and Z axes as a double array.
     */
    public double[] getOperatorXYZAxes(){
        if (operatorController.getHID().getType() != GenericHID.HIDType.kXInputGamepad) {
            return new double[]{
                    operatorController.getRawAxis(0),
                    operatorController.getRawAxis(1),
                    operatorController.getRawAxis(2)};
        }

        return new double[]{
                operatorController.getRawAxis(0),
                operatorController.getRawAxis(1),
                operatorController.getRawAxis(4)};
    }
}
