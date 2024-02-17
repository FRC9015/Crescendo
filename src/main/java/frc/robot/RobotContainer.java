// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static final SwerveSubsystem SWERVE = new SwerveSubsystem();
	public static final IntakeSubsystem INTAKE = new IntakeSubsystem();
	public static final ShooterSubsystem SHOOTER = new ShooterSubsystem();
	public static final Pigeon PIGEON = new Pigeon();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final LimelightInterface LIMELIGHT_INTERFACE = new LimelightInterface();

	SendableChooser<Command> pathChooser = new SendableChooser<>();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		Shuffleboard.getTab("Autonomous").add(pathChooser);

		Command swerveDriveCommand = SWERVE.driveCommand(
				() -> MathUtil.applyDeadband(-InputManager.getInstance().getDriverXYZAxes()[1], 0.15),
				() -> MathUtil.applyDeadband(-InputManager.getInstance().getDriverXYZAxes()[0], 0.15),
				() -> MathUtil.applyDeadband(-InputManager.getInstance().getDriverXYZAxes()[2], 0.15)
		);

		SWERVE.setDefaultCommand(swerveDriveCommand);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		//InputManager.getInstance().getDriverButton(InputManager.Button.LT_Button7).whileTrue(INTAKE.intakeNote());
		InputManager.getInstance().getDriverButton(InputManager.Button.LB_Button5).whileTrue(INTAKE.outtakeNote());
		InputManager.getInstance().getDriverButton(InputManager.Button.RB_Button6).whileTrue(INTAKE.intakeNote());

		InputManager.getInstance().getOperatorButton(InputManager.Button.LB_Button5).whileTrue(SHOOTER.shootNoteToSpeaker());
		InputManager.getInstance().getOperatorButton(InputManager.Button.RB_Button6).whileTrue(SHOOTER.shootNoteToAmp());
	}

	public Command getAutonomousCommand() {
		return pathChooser.getSelected();
	  }
}