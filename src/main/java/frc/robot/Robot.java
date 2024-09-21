// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Constants;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables


            //setUseTiming(false); // Run as fast as possible
            //String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            //Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            //Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }
        
            Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger
              .recordOutput(
                  "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
        // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.start();

        FollowPathCommand.warmupCommand().schedule();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();


        Logger.recordOutput("inWing", Constants.FieldConstants.WING.isIn(RobotContainer.SWERVE.getPose()));

        Logger.recordOutput("Pose/BotPose", RobotContainer.SWERVE.getPose());
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        m_robotContainer.disableRobot();
    }

    @Override
    public void disabledPeriodic() {
        //RobotContainer.SWERVE.updatePose();
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }


        // schedule the autonomous command (example)
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        if(RobotContainer.INTAKE.getHandoffStatus()){
            RobotContainer.INTAKE.handoff = false;
        }
        if(RobotContainer.SHOOTER.getShooterSensor()){
            RobotContainer.INTAKE.handoff = true;
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.enableRobot();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        //RobotContainer.SWERVE.updatePose();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
