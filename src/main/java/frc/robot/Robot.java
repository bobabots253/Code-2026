// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.autoUtil.AutoCommandPicker;
import frc.robot.util.fuelSimUtil.FuelSim;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private AutoCommandPicker[] autos = new AutoCommandPicker[Constants.AmountOfAutos];
  private Timer m_gcTimer = new Timer();

  // Flag to ensure we only apply the alliance offset once per DS connection
  @AutoLogOutput(key = "Robot/HasInitializedAlliancePose")
  private boolean hasInitializedAlliancePose = false;

  public Robot() {

    // Start GC timer
    m_gcTimer.start();

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible, ignore 50 hz limit
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());

    // Start AdvantageKit logger
    Logger.start();

    for (int i = 0; i < Constants.AmountOfAutos; i++) {
      autos[i] = new AutoCommandPicker(robotContainer);
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
    FullSubsystem.runAllPeriodicAfterScheduler();

    if (m_gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
    // Optionally call the garbage collector which manually suggest GC reduce memory usage
    // However, Java Garbage Collection should already be optimized to not have to call this
    // function manually.
    // Instead, reduce the amount of logging and induce static {} calls in individual subsystem
    // constants to dedicate single memory location

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Confirm these are correct
    // before using IMU modes 1,2,3...
    // LimelightHelpers.SetIMUMode(VisionConstants.cameraOrange, 0);
    LimelightHelpers.SetIMUMode(VisionConstants.cameraPurple, 0);
    LimelightHelpers.SetIMUMode(VisionConstants.cameraOrange, 0);
    LimelightHelpers.SetIMUMode(VisionConstants.cameraYellow, 0);
    LimelightHelpers.SetIMUMode(VisionConstants.cameraPink, 0);

    // Reset flag so it redoes its thingy if DS reconnects or alliance changes
    if (robotContainer.isAllianceHandledAlready()) {
      hasInitializedAlliancePose = true;
    } else {
      hasInitializedAlliancePose = false;
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    // Only apply nce per time DS connects to robot
    if (!hasInitializedAlliancePose && DriverStation.getAlliance().isPresent()) {
      robotContainer.applyAlliancePoseOffset();
      hasInitializedAlliancePose = true;
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // PathPlanner resets pose from path start
    // Note: Reset flag so if teleop follows without a disable, we don't re-apply
    hasInitializedAlliancePose = true;

    autonomousCommand = robotContainer.getAutonomousCommand();

    Command[] listOfCommands = new Command[Constants.AmountOfAutos];
    for (int i = 0; i < Constants.AmountOfAutos; i++) {
      listOfCommands[i] = autos[i].getCommand();
    }

    // schedule the autonomous command (example)
    // SequentialCommandGroup autoCommand = new SequentialCommandGroup(listOfCommands);

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(listOfCommands);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // Handle edge case where alliance data isn't recieved before enabled
    if (!hasInitializedAlliancePose) {
      robotContainer.applyAlliancePoseOffset();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    FuelSim.getInstance().updateSim();
  }

  /** Whether to display alerts related to hardware faults. */
  public static boolean showHardwareAlerts() {
    return Constants.getMode() != Mode.SIM && Timer.getTimestamp() > 30.0;
  }
}
