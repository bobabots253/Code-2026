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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.agitator.AgitatorIO;
import frc.robot.subsystems.agitator.AgitatorIOSim;
import frc.robot.subsystems.agitator.AgitatorIOSpark;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSpark;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOSim;
import frc.robot.subsystems.intake.pivot.PivotIOSpark;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOSim;
import frc.robot.subsystems.intake.roller.RollerIOSpark;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.kicker.KickerIOSpark;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.PassSide;
import frc.robot.subsystems.shooter.ShotCalculator.ShotMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOSpark;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
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
  private final PivotSubsystem pivotSubsystem;
  private final RollerSubsystem rollerSubsystem;
  private final AgitatorSubsystem agitatorSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final Vision vision;
  private final ShotCalculator shotCalculator;

  private Alliance lastAppliedAlliance = null;
  //   private final HoodSubsystem hoodSubsystem;
  //   private final FlywheelSubsystem flywheelSubsystem;
  //   private final ShooterSubsystem shooterSubsystem;

  // Dashboard Inputs
  //   private final LoggedDashboardChooser<Integer> clampVisionChooser =
  //       new LoggedDashboardChooser<>("Clamp Vision Estimates");

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

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
                swerveSubsystem::getRotation,
                swerveSubsystem::getChassisSpeeds,
                new VisionIOLimelight(
                    VisionConstants.cameraYellow,
                    swerveSubsystem::getRotation,
                    swerveSubsystem::getYawVelocityRate),
                new VisionIOLimelight(
                    VisionConstants.cameraPink,
                    swerveSubsystem::getRotation,
                    swerveSubsystem::getYawVelocityRate));

        shotCalculator = new ShotCalculator(swerveSubsystem);

        pivotSubsystem = new PivotSubsystem(new PivotIOSpark());
        rollerSubsystem = new RollerSubsystem(new RollerIOSpark());
        agitatorSubsystem = new AgitatorSubsystem(new AgitatorIOSpark());
        indexerSubsystem = new IndexerSubsystem(new IndexerIOSpark());
        kickerSubsystem = new KickerSubsystem(new KickerIOSpark());
        flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSpark());
        hoodSubsystem = new HoodSubsystem(new HoodIOSpark(), shotCalculator);

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
                    VisionConstants.cameraPlaceholder,
                    VisionConstants.cameraTransformToGreen,
                    swerveSubsystem::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraPlaceholder,
                    VisionConstants.cameraTransformToBlue,
                    swerveSubsystem::getPose));

        shotCalculator = new ShotCalculator(swerveSubsystem);

        pivotSubsystem = new PivotSubsystem(new PivotIOSim());
        rollerSubsystem = new RollerSubsystem(new RollerIOSim());
        agitatorSubsystem = new AgitatorSubsystem(new AgitatorIOSim());
        indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
        kickerSubsystem = new KickerSubsystem(new KickerIOSim());
        flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSim());
        hoodSubsystem = new HoodSubsystem(new HoodIOSim(), shotCalculator);

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

        pivotSubsystem = new PivotSubsystem(new PivotIO() {});
        rollerSubsystem = new RollerSubsystem(new RollerIO() {});
        agitatorSubsystem = new AgitatorSubsystem(new AgitatorIO() {});
        indexerSubsystem = new IndexerSubsystem(new IndexerIO() {});
        kickerSubsystem = new KickerSubsystem(new KickerIO() {});
        flywheelSubsystem = new FlywheelSubsystem(new FlywheelIO() {});
        hoodSubsystem = new HoodSubsystem(new HoodIO() {}, shotCalculator);

        break;
    }

    // ------- Intake Auto NamedCommands -------- \\

    NamedCommands.registerCommand("pivotDown", pivotSubsystem.deployCommand());
    NamedCommands.registerCommand("pivotUp", pivotSubsystem.stowCommand());
    NamedCommands.registerCommand("pivotShake", pivotSubsystem.runSaltAndPepperCommand());

    NamedCommands.registerCommand("rollerIntake", rollerSubsystem.intakeCommand());

    // ------- Agitator Auto NamedCommands -------- \\

    NamedCommands.registerCommand("agitatorIntake", agitatorSubsystem.intakeCommand());
    NamedCommands.registerCommand("agitatorIndex", agitatorSubsystem.indexCommand());

    // ------- Indexer Auto NamedCommands -------- \\

    NamedCommands.registerCommand("indexerIndex", indexerSubsystem.runCurrentCommand());

    // ------- Kicker Auto NamedCommands -------- \\

    NamedCommands.registerCommand("kickerIndex", kickerSubsystem.indexCommand());

    // ------- Shooter Auto NamedCommands -------- \\

    NamedCommands.registerCommand("flywheelLayup", flywheelSubsystem.runLayupCommand());
    NamedCommands.registerCommand(
        "flywheelDynamic",
        flywheelSubsystem.dynamicUpdatedShootCommand(
            () -> shotCalculator.getCorrectTargetVelocity()));
    NamedCommands.registerCommand("toggleWarm", flywheelSubsystem.toggleWarm());

    NamedCommands.registerCommand("hoodLayup", hoodSubsystem.runLayupCommand());
    NamedCommands.registerCommand(
        "hoodDynamic",
        hoodSubsystem.dynamicUpdatedShootCommand(
            () -> Units.degreesToRadians(shotCalculator.getCorrectedTargetAngle())));

    NamedCommands.registerCommand("toggleHub", shotCalculator.toggleShotMode(ShotMode.HUB));

    // ------- Drive Auto NamedCommands -------- \\

    NamedCommands.registerCommand(
        "driveHubLock",
        DriveCommands.joystickDriveAtAngle(
            swerveSubsystem,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> shotCalculator.getCorrectTargetRotation()));

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
    configureAutos();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   *
   * <p>Driver: - Indexer, Agitator, Kicker - Left Bumper - Angle Lock - Right Bumper Operator: -
   * Intake - Pivot, Roller, Unjam - Pivot, Left Bumper (Down), Right Bumper (Up) - Roller, Agitator
   * - X (Intake), B (Unjam) - Flywheel, Hood - Y (Queue Static Shot) - Hood - DPad Manual (Up,
   * Down) - Flywheel - Dpad Manual (Right)
   */
  private void configureButtonBindings() {

    // ------- Driver Controls -------- \\

    swerveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
            swerveSubsystem,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    driver
        .leftBumper()
        .whileTrue(indexerSubsystem.indexCommand())
        .whileTrue(kickerSubsystem.indexCommand())
        .whileTrue(agitatorSubsystem.indexCommand())
        .whileTrue(pivotSubsystem.runSaltAndPepperCommand());

    driver
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                swerveSubsystem,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> shotCalculator.getCorrectTargetRotation()));

    driver
        .b()
        .whileTrue(flywheelSubsystem.juggleCommand())
        .whileTrue(hoodSubsystem.juggleCommand());

    // Reset gyro to 0° when B button is pressed
    driver
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        swerveSubsystem.setPose(
                            new Pose2d(
                                swerveSubsystem.getPose().getTranslation(),
                                returnGlobalSwerveOffset())),
                    swerveSubsystem)
                .ignoringDisable(true));

    driver.povUp().whileTrue(hoodSubsystem.runDebuggingUpCommand());
    driver.povDown().whileTrue(hoodSubsystem.runDebuggingDownCommand());
    driver.povRight().whileTrue(flywheelSubsystem.runDebuggingVelocityCommand());

    // ------- Operator Controls -------- \\

    operator.leftBumper().onTrue(pivotSubsystem.deployCommand());
    operator.rightBumper().onTrue(pivotSubsystem.stowCommand());

    operator
        .x()
        .whileTrue(rollerSubsystem.intakeCommand())
        .whileTrue(agitatorSubsystem.intakeCommand());

    operator.b().whileTrue(rollerSubsystem.runUnjamCommand());

    operator
        .y()
        .whileTrue(
            flywheelSubsystem.dynamicUpdatedShootCommand(
                () -> shotCalculator.getCorrectTargetVelocity()));

    operator.a().onTrue(flywheelSubsystem.toggleWarm()); // Shifted from DPad Left

    operator
        .povUp()
        .onTrue(shotCalculator.toggleShotMode(ShotMode.HUB)); // Operator MUST get this right

    operator
        .povLeft()
        .onTrue(shotCalculator.togglePass(PassSide.CLOSE_LEFT)); // Operator MUST get this right
    operator
        .povRight()
        .onTrue(shotCalculator.togglePass(PassSide.CLOSE_RIGHT)); // Operator MUST get this right

    // ------ Debugging -------- \\

    // Default command, normal field-relative drive
    // swerveSubsystem.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         swerveSubsystem,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> -controller.getRightX()));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     swerveSubsystem.setPose(
    //                         new Pose2d(
    //                             swerveSubsystem.getPose().getTranslation(), new Rotation2d())),
    //                 swerveSubsystem)
    //             .ignoringDisable(true));
  }

  private void configureAutos() {

    // ------ Named Commands -------- \\
    // Autos created in PathPlanner UI are automatically pushed to AutoChooser
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * See: https://www.chiefdelphi.com/t/fatal-robot-code-crash/427074/26
   *
   * <p>Use this to clear the autonomous command connected to main {@link Robot} class.
   *
   * @return
   */
  //   public void killAutonomousBuilder(){
  //     autoChooser.
  //   }

  // Only use for Driver Reset Button Binding
  public Rotation2d returnGlobalSwerveOffset() {
    if (!DriverStation.getAlliance().isPresent()) {
      return new Rotation2d();
    } else {
      Alliance alliance = DriverStation.getAlliance().get();
      boolean isRed = alliance == Alliance.Red;
      Rotation2d allianceOffset =
          isRed ? Rotation2d.fromDegrees(180.0) : Rotation2d.fromDegrees(0.0);
      return allianceOffset;
    }
  }

  /*
   * Applies the alliance-relative pose offset to the swerve pose estimator.
   * Should be called once alliance is known by the DS (mainly from disabledPeriodic via Robot.java).
   *  Adds 180* on Red, 0* on Blue (relative to the gyro inital heading).
   */
  public void applyAlliancePoseOffset() {
    if (!DriverStation.getAlliance().isPresent()) return;

    Alliance alliance = DriverStation.getAlliance().get();
    boolean isRed = alliance == Alliance.Red;

    Rotation2d currentHeading = swerveSubsystem.getRotation();
    Rotation2d allianceOffset = isRed ? Rotation2d.fromDegrees(180.0) : Rotation2d.fromDegrees(0.0);

    swerveSubsystem.setPose(
        new Pose2d(
            swerveSubsystem.getPose().getTranslation(), currentHeading.plus(allianceOffset)));
    lastAppliedAlliance = alliance;
  }

  public boolean isAllianceHandledAlready() {
    if (!DriverStation.getAlliance().isPresent()) return false;
    if (lastAppliedAlliance == null) return false;
    return lastAppliedAlliance == DriverStation.getAlliance().get();
  }
}
