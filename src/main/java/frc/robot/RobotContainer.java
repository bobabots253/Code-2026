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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem swerveSubsystem;
  private final Vision vision;

  //labubu

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        swerveSubsystem =
            new SwerveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        vision =
            new Vision(
                swerveSubsystem::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.FrontLeftLL, swerveSubsystem::getRotation),
                new VisionIOLimelight(VisionConstants.FrontRightLL, swerveSubsystem::getRotation));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        swerveSubsystem =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision =
            new Vision(
                swerveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.FrontLeftLL,
                    VisionConstants.robotToFrontLeftLL,
                    swerveSubsystem::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.FrontRightLL,
                    VisionConstants.robotToFrontRightLL,
                    swerveSubsystem::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        swerveSubsystem =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(swerveSubsystem::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(swerveSubsystem));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(swerveSubsystem));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        swerveSubsystem.sysIdDynamic(
            SysIdRoutine.Direction.kReverse)); // <-- Goofy Compile Time Syntax  Error

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    swerveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
            swerveSubsystem,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                swerveSubsystem,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(swerveSubsystem::stopWithX, swerveSubsystem));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        swerveSubsystem.setPose(
                            new Pose2d(
                                swerveSubsystem.getPose().getTranslation(), new Rotation2d())),
                    swerveSubsystem)
                .ignoringDisable(true));

    // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // controller
    //     .y()
    //     .whileTrue(
    //         Commands.startRun(
    //             () -> {
    //               aimController.reset();
    //             },
    //             () -> {
    //                 swerveSubsystem.run(0.0,
    // aimController.calculate(vision.getTargetX(0).getRadians()));
    //             },
    //             swerveSubsystem));
    // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
