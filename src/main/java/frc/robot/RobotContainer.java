// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.Handoff;
import frc.robot.Commands.LimelightDrive;
import frc.robot.Commands.AutoAim;
import frc.robot.Commands.Presets.AmpPreset;
import frc.robot.Commands.Presets.PassNotePreset;
import frc.robot.Commands.Presets.SubwooferPreset;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static final SwerveSubsystem SWERVE = new SwerveSubsystem();
	public static final Pigeon PIGEON = new Pigeon();
	public static final PoseEstimator POSE_ESTIMATOR =
			new PoseEstimator(SWERVE, PIGEON, new Pose2d(1, 1, PIGEON.getYawAsRotation2d()));

	public static final PivotSubsystem PIVOT = new PivotSubsystem();
	public static final IntakeSubsystem INTAKE = new IntakeSubsystem();

	public static final ShooterSubsystem SHOOTER = new ShooterSubsystem();

	public static final LimelightInterface LIMELIGHT_INTERFACE = new LimelightInterface();
	public static final LEDSubsystem LED_SUBSYSTEM = new LEDSubsystem(INTAKE,SHOOTER);
	public static final HangerSubsystem HANGER = new HangerSubsystem();
	public static final AmpSubsystem AMP = new AmpSubsystem();

	SendableChooser<Command> autoChooser = new SendableChooser<>();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
		NamedCommands.registerCommand("shootNote", SHOOTER.autoShootNoteToSpeaker(AMP));
		NamedCommands.registerCommand("shootLimelight", SHOOTER.autoShootNoteLimelight(AMP));
		NamedCommands.registerCommand("intakeNote", INTAKE.autoIntakeNote());
		NamedCommands.registerCommand("revShooter", SHOOTER.revShooter());
		NamedCommands.registerCommand("outtakeNote", INTAKE.outtakeNote());
		NamedCommands.registerCommand("stopSpeakerShooter", SHOOTER.stopShooter());
		NamedCommands.registerCommand("intake", new Handoff(INTAKE,AMP).until(SHOOTER::getShooterSensor));
		NamedCommands.registerCommand("stopIntake",INTAKE.stopIntake());
		NamedCommands.registerCommand("ampShoot", AMP.shootNoteToAmp());
		NamedCommands.registerCommand("pivotToIntake", PIVOT.movePivotToIntake());
		NamedCommands.registerCommand("backwardShooter", SHOOTER.autoBackwardShooter());
		NamedCommands.registerCommand("autoAim", PIVOT.autoAutoAim());
		NamedCommands.registerCommand("pivotToSubWoofer", PIVOT.movePivotToSubWoofer());

		SWERVE.setDefaultCommand(new DefaultDrive());
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
				// Driver Bindings
				InputManager.getInstance().getDriverButton(InputManager.Button.B_Button2).whileTrue(INTAKE.outtakeNote());
				InputManager.getInstance().getDriverButton(InputManager.Button.RB_Button6).whileTrue(new Handoff(INTAKE,AMP).until(SHOOTER::getShooterSensor).andThen(SHOOTER::setIdleShooterSpeeds));
				InputManager.getInstance().getDriverButton(InputManager.Button.Y_Button4).whileTrue(new InstantCommand(POSE_ESTIMATOR::updatePoseEstimator).repeatedly());
				InputManager.getInstance().getDriverButton(InputManager.Button.X_Button3).onTrue(new InstantCommand(PIGEON::zeroYaw));
				InputManager.getInstance().getDriverButton(InputManager.Button.LB_Button5).onTrue(SWERVE.slowModeOn()).onFalse(SWERVE.slowModeOff());
				InputManager.getInstance().getDriverButton(InputManager.Button.A_Button1).onTrue(PIVOT.printPivotAngle());
				InputManager.getInstance().getDriverPOV(0).onTrue(HANGER.hangerUP());
				InputManager.getInstance().getDriverPOV(180).onTrue(HANGER.hangerDOWN());
				
				// Operator Bindings
				InputManager.getInstance().getOperatorButton(InputManager.Button.RB_Button6).whileTrue(AMP.shootNoteToAmp());
				InputManager.getInstance().getOperatorButton(InputManager.Button.LB_Button5).whileTrue(SHOOTER.shootNoteToSpeaker());
				InputManager.getInstance().getOperatorButton(InputManager.Button.B_Button2).whileTrue((new AutoAim()).alongWith(new LimelightDrive()));
				InputManager.getInstance().getOperatorPOV(270).whileTrue(AMP.ampIntake());
				InputManager.getInstance().getOperatorPOV(90).whileTrue(SHOOTER.shooterBackward());
				InputManager.getInstance().getOperatorPOV(0).whileTrue(PIVOT.raisePivot());
				InputManager.getInstance().getOperatorPOV(180).whileTrue(PIVOT.lowerPivot());		
				// Operator Presets
				InputManager.getInstance().getOperatorButton(InputManager.Button.Y_Button4).whileTrue(new AmpPreset());
				InputManager.getInstance().getOperatorButton(InputManager.Button.A_Button1).whileTrue(new SubwooferPreset());
				InputManager.getInstance().getOperatorButton(InputManager.Button.X_Button3).whileTrue(new PassNotePreset());
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	  }

	public void disableRobot(){
		PIVOT.SubWoofer();
		INTAKE.stopIntake();
		SHOOTER.stopSpeakerShooterMotors();
		AMP.stopAmp();
	}

	public void enableRobot(){
		PIVOT.intake();
		INTAKE.stopIntake();
		SHOOTER.setIdleShooterSpeeds();
		AMP.stopAmp();
		
	}

	

	public static boolean IsRed(){
		var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
				}
				return false;
	}
}
