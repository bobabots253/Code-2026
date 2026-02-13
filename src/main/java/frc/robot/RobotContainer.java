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
import frc.robot.subsystems.intake.roller.RollerIOSpark;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
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
  private final ShotCalculator shotCalculator;
  //   private final HoodSubsystem hoodSubsystem;
  //   private final FlywheelSubsystem flywheelSubsystem;
  //   private final ShooterSubsystem shooterSubsystem;
  private final RollerSubsystem rollerSubsystem;

  // Dashboard Inputs
  private final LoggedDashboardChooser<Integer> clampVisionChooser =
      new LoggedDashboardChooser<>("Clamp Vision Estimates");

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    clampVisionChooser.addDefaultOption("Locked", 10);
    clampVisionChooser.addOption("Unlocked | Purple", 0);
    clampVisionChooser.addOption("Unlocked | Orange", 1);
    clampVisionChooser.addOption("Unlocked | Green", 2);
    clampVisionChooser.addOption("Unlocked | Blue", 3);

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
                swerveSubsystem::getRotation,
                swerveSubsystem::getChassisSpeeds,
                new VisionIOLimelight(VisionConstants.cameraPurple, swerveSubsystem::getRotation),
                new VisionIOLimelight(VisionConstants.cameraOrange, swerveSubsystem::getRotation),
                new VisionIOLimelight(VisionConstants.cameraGreen, swerveSubsystem::getRotation),
                new VisionIOLimelight(VisionConstants.cameraBlue, swerveSubsystem::getRotation));

        shotCalculator = new ShotCalculator(swerveSubsystem);

        // hoodSubsystem = new HoodSubsystem(new frc.robot.subsystems.shooter.hood.HoodIOSpark());
        // flywheelSubsystem =
        //     new FlywheelSubsystem(new frc.robot.subsystems.shooter.flywheel.FlywheelIOSpark());
        // shooterSubsystem =
        //     new ShooterSubsystem(
        //         flywheelSubsystem,
        //         hoodSubsystem,
        //         shotCalculator,
        //         swerveSubsystem::getPose,
        //         swerveSubsystem::getChassisSpeeds);
        rollerSubsystem = new RollerSubsystem(new RollerIOSpark());

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
                swerveSubsystem::getRotation,
                swerveSubsystem::getChassisSpeeds,
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraPurple,
                    VisionConstants.cameraTransformToPurple,
                    swerveSubsystem::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraOrange,
                    VisionConstants.cameraTransformToOrange,
                    swerveSubsystem::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraGreen,
                    VisionConstants.cameraTransformToGreen,
                    swerveSubsystem::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraBlue,
                    VisionConstants.cameraTransformToBlue,
                    swerveSubsystem::getPose));

        shotCalculator = new ShotCalculator(swerveSubsystem);
        // hoodSubsystem = new HoodSubsystem(new HoodIOSim());
        // flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSim());
        // shooterSubsystem =
        //     new ShooterSubsystem(
        //         flywheelSubsystem,
        //         hoodSubsystem,
        //         shotCalculator,
        //         swerveSubsystem::getPose,
        //         swerveSubsystem::getChassisSpeeds);
        rollerSubsystem = new RollerSubsystem(new RollerIOSpark());

        // configureFuelSim();
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
            new Vision(
                swerveSubsystem::addVisionMeasurement,
                swerveSubsystem::getRotation,
                swerveSubsystem::getChassisSpeeds,
                new VisionIO() {},
                new VisionIO() {});

        shotCalculator = new ShotCalculator(swerveSubsystem);
        // hoodSubsystem = new HoodSubsystem(new HoodIOSim());
        // flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSim());
        // shooterSubsystem =
        //     new ShooterSubsystem(
        //         new FlywheelSubsystem(new FlywheelIO() {}),
        //         new HoodSubsystem(new HoodIO() {}),
        //         new ShotCalculator(swerveSubsystem),
        //         swerveSubsystem::getPose,
        //         swerveSubsystem::getChassisSpeeds);
        rollerSubsystem = new RollerSubsystem(new RollerIOSpark());
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

  //   public Command rollerSpeed(double speed) {
  //     return new InstantCommand(rollerSubsystem, ()-> rollerSubsystem.setSpeed());
  //   }

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

    // Lock to Hub when A button is held
    controller
        .a()
        .whileTrue(rollerSubsystem.setVoltage(-12.0))
        .whileFalse(rollerSubsystem.setVoltage(0));
    // .whileFalse(rollerSubsystem.setVoltage(0));

    // Shoot on the fly when X button is pressed
    // controller.x().whileTrue(shooterSubsystem.simShootOnTheFlyCommand());

    // Shoot on the fly while Y button is held, With drive control
    // controller
    //     .y()
    //     .whileTrue(
    //         Commands.parallel(
    //             DriveCommands.joystickDriveAtAngle(
    //                 swerveSubsystem,
    //                 () -> -controller.getLeftY(),
    //                 () -> -controller.getLeftX(),
    //                 () -> shotCalculator.getCorrectTargetRotation()),
    //             shooterSubsystem.simShootOnTheFlyCommand()));

    // Reset gyro to 0° when B button is pressed
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
  }

  //   public void configureFuelSim() {
  //     FuelSim instance = FuelSim.getInstance();
  //     instance.spawnStartingFuel();
  //     instance.registerRobot(
  //         SwerveConstants.ROBOT_LENGTH.in(Meters),
  //         SwerveConstants.ROBOT_WIDTH.in(Meters),
  //         SwerveConstants.BUMPER_HEIGHT.in(Meters),
  //         swerveSubsystem::getPose,
  //         swerveSubsystem::getChassisSpeeds);
  //     instance.registerIntake(
  //         SwerveConstants.ROBOT_LENGTH.div(2).in(Meters),
  //         SwerveConstants.ROBOT_LENGTH.div(2).plus(Inches.of(5)).in(Meters),
  //         SwerveConstants.ROBOT_WIDTH.div(2).unaryMinus().in(Meters),
  //         SwerveConstants.ROBOT_WIDTH.div(2).in(Meters),
  //         () -> true && shooterSubsystem.simAbleToIntake(),
  //         shooterSubsystem::simIntake);

  // instance.start();
  // Commands.runOnce(
  //         () -> {
  //           FuelSim.getInstance().clearFuel();
  //           FuelSim.getInstance().spawnStartingFuel();
  //         })
  //     .schedule();
  //   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Returns whether vision estimates should be clamped.
   *
   * @return true if vision estimates should be clamped. Enabled by default.
   */
  public Integer enableVisionClamp() {
    return clampVisionChooser.get();
  }
}
