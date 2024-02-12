package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Constants.OperatorConstants;

/**
 * This handles ALL input from the controller.
 * Supports Joysticks and Xbox Controllers.
 * DON'T TOUCH THIS CLASS WITHOUT ME!
 * - Matthew
 */
public class InputManager {
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

    private InputManager(){
        driveController = new CommandGenericHID(OperatorConstants.driverControllerPort);
        operatorController = new CommandGenericHID(OperatorConstants.operatorControllerPort);
    }

    public static InputManager getInstance(){
        if (Instance == null){
            Instance = new InputManager();
        }

        return Instance;
    }

    public Trigger getDriverButton(Button button){
        if (driveController.getHID().getType() != GenericHID.HIDType.kXInputGamepad) {
            return driveController.button(button.buttonID);
        }

        double pressThreshold = 0.1;
        return switch (button) {
            case LT_Button7 -> driveController.axisGreaterThan(2, pressThreshold);
            case RT_Button8 -> driveController.axisGreaterThan(3, pressThreshold);
            default -> driveController.button(button.buttonID);
        };
    }

    public Trigger getOperatorButton(Button button){
        if (operatorController.getHID().getType() != GenericHID.HIDType.kXInputGamepad) {
            return operatorController.button(button.buttonID);
        }

        double pressThreshold = 0.1;
        return switch (button) {
            case LT_Button7 -> operatorController.axisGreaterThan(2, pressThreshold);
            case RT_Button8 -> operatorController.axisGreaterThan(3, pressThreshold);
            default -> operatorController.button(button.buttonID);
        };
    }

    public double[] getDriverXYZAxes(){
        if (driveController.getHID().getType() != GenericHID.HIDType.kXInputGamepad) {
            return new double[]{
                    driveController.getRawAxis(0),
                    driveController.getRawAxis(1),
                    driveController.getRawAxis(2)};
        }

        return new double[]{
                driveController.getRawAxis(0),
                driveController.getRawAxis(1),
                driveController.getRawAxis(4)};
    }

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
