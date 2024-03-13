// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAim;
import frc.robot.commands.Handoff;
import frc.robot.commands.Presets.*;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final Field2d pathField;
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

	public static final SwerveSubsystem SWERVE = new SwerveSubsystem();
	public static final IntakeSubsystem INTAKE = new IntakeSubsystem();
	public static final ShooterSubsystem SHOOTER = new ShooterSubsystem();
	public static final PivotSubsystem PIVOT = new PivotSubsystem();
	public static final CameraSubsystem CAMERA_SUBSYSTEM = new CameraSubsystem();



	// Replace with CommandPS4Controller or CommandJoystick if needed
	public static final LimelightInterface LIMELIGHT_INTERFACE = new LimelightInterface();
	

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


		Command swerveDriveCommand = SWERVE.driveCommand( //TODO Add slew rate limiting to inputs
				() -> MathUtil.applyDeadband(-InputManager.getInstance().getDriverXYZAxes()[1], 0.15),
				() -> MathUtil.applyDeadband(-InputManager.getInstance().getDriverXYZAxes()[0], 0.15),
				() -> MathUtil.applyDeadband(-InputManager.getInstance().getDriverXYZAxes()[2], 0.15)
		);

		SWERVE.setDefaultCommand(swerveDriveCommand);
		SWERVE.setupPathPlanner();

		autoChooser = AutoBuilder.buildAutoChooser(); //might look for stuff in the folder; try deleting if no work.

		Shuffleboard.getTab("Autonomous").add(autoChooser);



		pathField = new Field2d();

		SmartDashboard.putData("Field", pathField);


		// Logging callback for current robot pose
        // Do whatever you want with the pose here
        PathPlannerLogging.setLogCurrentPoseCallback(pathField::setRobotPose);

		// Logging callback for target robot pose
		PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			pathField.getObject("target pose").setPose(pose);
		});

		// Logging callback for the active path, this is sent as a list of poses
		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			// Do whatever you want with the poses here
			pathField.getObject("path").setPoses(poses);
		});

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
		InputManager.getInstance().getDriverButton(InputManager.Button.LB_Button5).whileTrue(INTAKE.outtakeNote());
		InputManager.getInstance().getDriverButton(InputManager.Button.RB_Button6).whileTrue(new Handoff(INTAKE,SHOOTER));
		InputManager.getInstance().getDriverButton(InputManager.Button.Y_Button4).onTrue(SWERVE.zeroYaw());

		// Operator Bindings
		InputManager.getInstance().getOperatorButton(InputManager.Button.RB_Button6).whileTrue(SHOOTER.shootNoteToAmp());
		InputManager.getInstance().getOperatorButton(InputManager.Button.LB_Button5).whileTrue(SHOOTER.shootNoteToSpeaker());
		InputManager.getInstance().getOperatorButton(InputManager.Button.B_Button2).whileTrue(new AutoAim());
		InputManager.getInstance().getOperatorPOV(270).whileTrue(SHOOTER.ampIntake());
		InputManager.getInstance().getOperatorPOV(90).whileTrue(SHOOTER.shooterBackward());
		
				
		// Operator Presets
		
		InputManager.getInstance().getOperatorButton(InputManager.Button.Y_Button4).whileTrue(new AmpPreset());
		InputManager.getInstance().getOperatorButton(InputManager.Button.A_Button1).whileTrue(new SubWooferPreset());
		InputManager.getInstance().getOperatorPOV(0).onTrue(new InstantCommand(PIVOT::passNotePreset));
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