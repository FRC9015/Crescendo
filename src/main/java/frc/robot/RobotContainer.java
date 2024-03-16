// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.AutoAim;
import frc.robot.Commands.Handoff;
import frc.robot.Commands.Presets.AmpPreset;
import frc.robot.Commands.Presets.PassNotePreset;
import frc.robot.Commands.Presets.SubWooferPreset;
import frc.robot.Constants.Constants.OperatorConstants;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static final SwerveSubsystem SWERVE = new SwerveSubsystem();
	//public static final IntakeSubsystem INTAKE = new IntakeSubsystem();
	public static final Pigeon PIGEON = new Pigeon();
	public static final PoseEstimator POSE_ESTIMATOR =
			new PoseEstimator(SWERVE, PIGEON, new Pose2d(1, 1, PIGEON.getYawAsRotation2d()));

	public static final PivotSubsystem PIVOT = new PivotSubsystem();
	public static final ShooterSubsystem SHOOTER = new ShooterSubsystem();
	public static final IntakeSubsystem INTAKE = new IntakeSubsystem();
	public static final CameraSubsystem CAMERA = new CameraSubsystem();
	public static final LimelightInterface LIMELIGHT_INTERFACE = new LimelightInterface();

	public static final CommandXboxController driveController =
			new CommandXboxController(OperatorConstants.driverControllerPort);

	SendableChooser<Command> autoChooser = new SendableChooser<>();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
		NamedCommands.registerCommand("shootNote", SHOOTER.autoShootNoteToSpeaker());
		NamedCommands.registerCommand("intakeNote", INTAKE.autoIntakeNote());
		NamedCommands.registerCommand("outtakeNote", INTAKE.outtakeNote());
		NamedCommands.registerCommand("stopSpeakerShooter", SHOOTER.stopShooter());
		NamedCommands.registerCommand("intakeAmp", SHOOTER.autoAmpIntake());
		NamedCommands.registerCommand("stopIntake",INTAKE.stopIntake());
		NamedCommands.registerCommand("ampShoot", SHOOTER.shootNoteToAmp());
		NamedCommands.registerCommand("pivotToIntake", PIVOT.movePivotToIntake());
		NamedCommands.registerCommand("backwardShooter", SHOOTER.autoBackwardShooter());
		NamedCommands.registerCommand("autoAim", new AutoAim());
		NamedCommands.registerCommand("pivotToSubWoofer", PIVOT.movePivotToSubWoofer());
		
		configureBindings();

		SWERVE.setUpPathPlanner();
		autoChooser = AutoBuilder.buildAutoChooser();
		Shuffleboard.getTab("Autonomous").add(autoChooser);

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

				// Driver Bindings
				InputManager.getInstance().getDriverButton(InputManager.Button.LB_Button5).whileTrue(INTAKE.outtakeNote());
				InputManager.getInstance().getDriverButton(InputManager.Button.RB_Button6).and(SHOOTER.getSensor()).whileTrue(new Handoff(INTAKE,SHOOTER));
				
		
				// Operator Bindings
				InputManager.getInstance().getOperatorButton(InputManager.Button.RB_Button6).whileTrue(SHOOTER.shootNoteToAmp());
				InputManager.getInstance().getOperatorButton(InputManager.Button.LB_Button5).whileTrue(SHOOTER.shootNoteToSpeaker());
				InputManager.getInstance().getOperatorButton(InputManager.Button.B_Button2).whileTrue(new AutoAim());
				InputManager.getInstance().getOperatorPOV(270).whileTrue(SHOOTER.ampIntake());
				InputManager.getInstance().getOperatorPOV(90).whileTrue(SHOOTER.shooterBackward());
				
				// Operator Presets
				InputManager.getInstance().getOperatorButton(InputManager.Button.Y_Button4).whileTrue(new AmpPreset());
				InputManager.getInstance().getOperatorButton(InputManager.Button.A_Button1).whileTrue(new SubWooferPreset());
				InputManager.getInstance().getOperatorPOV(0).onTrue(new PassNotePreset());
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	  }

	public void disableRobot(){
		PIVOT.SubWoofer();
	}

	public void enableRobot(){
		PIVOT.intake();
	}
}