// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.Presets.RingTossPreset;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.*;
import frc.robot.Commands.Presets.AmpPreset;
import frc.robot.Commands.Presets.PassNotePreset;
import frc.robot.Commands.Presets.SubwooferPreset;
import frc.robot.Constants.Constants;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.Constants.Constants.FieldConstants;

public class RobotContainer {
    // Subsystems
    public static final SwerveSubsystem SWERVE = TunerConstants.DriveTrain;
    public static final Pigeon PIGEON = new Pigeon();
    public static final PivotSubsystem PIVOT = new PivotSubsystem();
    public static final IntakeSubsystem INTAKE = new IntakeSubsystem();
    public static final ShooterSubsystem SHOOTER = new ShooterSubsystem();
    public static final LimelightInterface LIMELIGHT_INTERFACE = new LimelightInterface();
    public static final LEDSubsystem LED_SUBSYSTEM = new LEDSubsystem(INTAKE, SHOOTER);
    public static final HangerSubsystem HANGER = new HangerSubsystem();
    public static final AmpSubsystem AMP = new AmpSubsystem();

    // Controllers
    private final CommandXboxController driverController = 
        new CommandXboxController(Constants.OperatorConstants.driverControllerPort);
    private final CommandXboxController operatorController = 
        new CommandXboxController(Constants.OperatorConstants.operatorControllerPort);

    // Autonomous
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureSubsystemDefaults();
        configureNamedCommands();
        configureBindings();
        
        SWERVE.setUpPathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add(autoChooser);
    }

    private void configureSubsystemDefaults() {
        SWERVE.setDefaultCommand(new DefaultDrive(
            SWERVE,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getHID().getRightBumper()
        ));
        
        SHOOTER.setDefaultCommand(new MaintainShooterIdle(SHOOTER));
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("shootNote", SHOOTER.autoShootNoteToSpeaker(AMP));
        NamedCommands.registerCommand("shootLimelight", SHOOTER.autoShootNoteLimelight(AMP));
        NamedCommands.registerCommand("intakeNote", INTAKE.autoIntakeNote());
        NamedCommands.registerCommand("revShooter", SHOOTER.revShooter());
        NamedCommands.registerCommand("outtakeNote", INTAKE.outtakeNote());
        NamedCommands.registerCommand("stopSpeakerShooter", SHOOTER.stopShooter());
        NamedCommands.registerCommand("intake", new Handoff(INTAKE, AMP).until(SHOOTER::getShooterSensor));
        NamedCommands.registerCommand("intakeTimeout", new Handoff(INTAKE, AMP).until(SHOOTER::getShooterSensor).withTimeout(2).until(() -> INTAKE.handoff));
        NamedCommands.registerCommand("stopIntake", INTAKE.stopIntake());
        NamedCommands.registerCommand("ampShoot", AMP.shootNoteToAmp());
        NamedCommands.registerCommand("pivotToIntake", PIVOT.movePivotToIntake());
        NamedCommands.registerCommand("backwardShooter", SHOOTER.autoBackwardShooter());
        NamedCommands.registerCommand("autoAim", PIVOT.autoAutoAim());
        NamedCommands.registerCommand("pivotToSubWoofer", PIVOT.movePivotToSubWooferAuto());
        NamedCommands.registerCommand("LimelightDrive", new AutoDrive());
    }

    private void configureBindings() {
        // Driver Controls
        driverController.leftBumper().whileTrue(INTAKE.outtakeNote());
        driverController.rightBumper().whileTrue(
            new Handoff(INTAKE, AMP)
                .until(SHOOTER::getShooterSensor)
                .andThen(SHOOTER::setIdleShooterSpeeds));
        
        driverController.x().onTrue(new InstantCommand(PIGEON::zeroYaw));
        driverController.leftTrigger(0.5).onTrue(SWERVE.slowModeOn()).onFalse(SWERVE.slowModeOff());
        driverController.a().onTrue(PIVOT.printPivotAngle());

        // Operator Controls
        operatorController.rightBumper().whileTrue(AMP.shootNoteToAmp());
        operatorController.leftBumper().whileTrue(SHOOTER.shootNoteToSpeaker());
        operatorController.b().toggleOnTrue(SHOOTER.setRandomMode());
        
        operatorController.povDown().whileTrue(AMP.ampIntake());
        operatorController.povRight().whileTrue(SHOOTER.shooterBackward());
        operatorController.povUp().whileTrue(PIVOT.raisePivot());
        operatorController.povDown().whileTrue(PIVOT.lowerPivot());
        
        operatorController.y().whileTrue(new AmpPreset());
        operatorController.a().whileTrue(new RingTossPreset());
        operatorController.x().whileTrue(new PassNotePreset());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // Alliance Color Detection
    public static boolean IsRed() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Red)
            .orElse(false);
    }

    // Robot State Management
    public void disableRobot() {
        Logger.recordOutput("Robot/Disabled", true);
        CommandScheduler.getInstance().cancelAll();
    }

    public void enableRobot() {
        Logger.recordOutput("Robot/Disabled", false);
        SWERVE.zeroGyro();
        INTAKE.resetIntakeEncoder();
    }

    // PID Telemetry
    public static void logPID(String name, PIDController pid) {
        Logger.recordOutput("PID/" + name + "/P", pid.getP());
        Logger.recordOutput("PID/" + name + "/I", pid.getI());
        Logger.recordOutput("PID/" + name + "/D", pid.getD());
        Logger.recordOutput("PID/" + name + "/Setpoint", pid.getSetpoint());
        Logger.recordOutput("PID/" + name + "/PositionError", pid.getPositionError());
    }
}
