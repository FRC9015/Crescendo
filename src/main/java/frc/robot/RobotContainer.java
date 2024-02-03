// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Constants.SwerveConstants.angularSpeed;
import static frc.robot.RobotContainer.inputManager;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SelfDrive.AmpSelfDrive;
import frc.robot.subsystems.SelfDrive.SpeakerSelfDrive;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.InputManager;

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
	public static final Pigeon PIGEON = new Pigeon();
	public static final PoseEstimator POSE_ESTIMATOR =
		new PoseEstimator(SWERVE, PIGEON, new Pose2d(1, 1, PIGEON.getYawAsRotation2d()));

	public static final InputManager inputManager = InputManager.getInstance();
	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final LimelightInterface LIMELIGHT_INTERFACE = new LimelightInterface();

	SendableChooser<Command> pathChooser = new SendableChooser<>();
	
	private final AmpSelfDrive AmpSelfDrive = new AmpSelfDrive(inputManager.getDriverController(), LIMELIGHT_INTERFACE, SWERVE,PIGEON);
	private final SpeakerSelfDrive SpeakerSelfDrive = new SpeakerSelfDrive(inputManager.getDriverController(), LIMELIGHT_INTERFACE, SWERVE);
	

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		Shuffleboard.getTab("Autonomous").add(pathChooser);
		
		pathChooser.addOption("Path 1", SWERVE.followPathCommandAuto("Example Path"));
		pathChooser.addOption("Path 2", SWERVE.followPathCommandAuto("New Path"));

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
		SWERVE.setDefaultCommand(new DefaultDrive());

		InputManager.getInstance().init(
				new InputManager.ButtonMap(InputManager.Button.LT_Button7, INTAKE.intakeNote()),
				new InputManager.ButtonMap(InputManager.Button.A_Button1, SWERVE.printOffsets()),
				new InputManager.ButtonMap(InputManager.Button.X_Button3, new InstantCommand(PIGEON::zeroYaw)),
				new InputManager.ButtonMap(InputManager.Button.B_Button2, new InstantCommand(POSE_ESTIMATOR::resetOdometry))
		);
	}

	public Command getAutonomousCommand() {
		return pathChooser.getSelected();
	  }
}