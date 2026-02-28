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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    // clampVisionChooser.addDefaultOption("Locked", 10);
    // clampVisionChooser.addOption("Unlocked | Purple", 0);
    // clampVisionChooser.addOption("Unlocked | Orange", 1);
    // clampVisionChooser.addOption("Unlocked | Green", 2);
    // clampVisionChooser.addOption("Unlocked | Blue", 3);

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
                // new VisionIOLimelight(VisionConstants.cameraPurple,
                // swerveSubsystem::getRotation),
                // new VisionIOLimelight(VisionConstants.cameraOrange,
                // swerveSubsystem::getRotation),
                new VisionIOLimelight(VisionConstants.cameraYellow, swerveSubsystem::getRotation),
                new VisionIOLimelight(VisionConstants.cameraPink, swerveSubsystem::getRotation));

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
        pivotSubsystem = new PivotSubsystem(new PivotIOSpark());
        rollerSubsystem = new RollerSubsystem(new RollerIOSpark());
        agitatorSubsystem = new AgitatorSubsystem(new AgitatorIOSpark());
        indexerSubsystem = new IndexerSubsystem(new IndexerIOSpark());
        kickerSubsystem = new KickerSubsystem(new KickerIOSpark());
        flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSpark());
        hoodSubsystem = new HoodSubsystem(new HoodIOSpark());

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
        // hoodSubsystem = new HoodSubsystem(new HoodIOSim());
        // flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSim());
        // shooterSubsystem =
        //     new ShooterSubsystem(
        //         flywheelSubsystem,
        //         hoodSubsystem,
        //         shotCalculator,
        //         swerveSubsystem::getPose,
        //         swerveSubsystem::getChassisSpeeds);
        pivotSubsystem = new PivotSubsystem(new PivotIOSim());
        rollerSubsystem = new RollerSubsystem(new RollerIOSim());
        agitatorSubsystem = new AgitatorSubsystem(new AgitatorIOSim());
        indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
        kickerSubsystem = new KickerSubsystem(new KickerIOSim());
        flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSim());
        hoodSubsystem = new HoodSubsystem(new HoodIOSim());

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
        pivotSubsystem = new PivotSubsystem(new PivotIO() {});
        rollerSubsystem = new RollerSubsystem(new RollerIO() {});
        agitatorSubsystem = new AgitatorSubsystem(new AgitatorIO() {});
        indexerSubsystem = new IndexerSubsystem(new IndexerIO() {});
        kickerSubsystem = new KickerSubsystem(new KickerIO() {});
        flywheelSubsystem = new FlywheelSubsystem(new FlywheelIO() {});
        hoodSubsystem = new HoodSubsystem(new HoodIO() {});

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
   *
   * <p>Driver: - Indexer, Agitator, Kicker - Left Bumper - Angle Lock - Right Bumper Operator: -
   * Intake - Pivot, Roller, Unjam - Pivot, Left Bumper (Down), Right Bumper (Up) - Roller, Agitator
   * - X (Intake), B (Unjam) - Flywheel, Hood - Y (Queue Static Shot) - Hood - DPad Manual (Up,
   * Down) - Flywheel - Dpad Manual (Right)
   */
  private void configureButtonBindings() {

    // ------- DT

    swerveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
            swerveSubsystem,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    driver
        .leftBumper()
        .whileTrue(indexerSubsystem.runCurrentCommand())
        .whileTrue(kickerSubsystem.indexCommand())
        .whileTrue(agitatorSubsystem.indexCommand());

    driver
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                swerveSubsystem,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> shotCalculator.getFieldToHubAngle()));

    operator.leftBumper().onTrue(pivotSubsystem.deployCommand());
    operator.rightBumper().onTrue(pivotSubsystem.stowCommand());

    operator
        .x()
        .whileTrue(rollerSubsystem.intakeCommand())
        .whileTrue(agitatorSubsystem.intakeCommand());

    operator.b().whileTrue(rollerSubsystem.runUnjamCommand());

    operator
        .y()
        .whileTrue(flywheelSubsystem.runStaticVelocitCommand())
        .whileTrue(hoodSubsystem.runStaticAngleCommand());

    operator.povDown().whileTrue(hoodSubsystem.runDebuggingVoltageDownCommand());
    operator.povUp().whileTrue(hoodSubsystem.runDebuggingVoltageUpCommand());

    operator.povRight().whileTrue(flywheelSubsystem.runDebuggingVelocityCommand());

    // ------ Debugging

    // Default command, normal field-relative drive
    // swerveSubsystem.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         swerveSubsystem,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> -controller.getRightX()));

    // controller.a().onTrue(pivotSubsystem.deployCommand());

    // controller
    //     .b()
    //     .whileTrue(flywheelSubsystem.runDebuggingVelocityCommand())
    //     .whileTrue(hoodSubsystem.runDebuggingCommand());

    // controller.y().onTrue(pivotSubsystem.stowCommand());

    // controller
    //     .x()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             swerveSubsystem,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> shotCalculator.getFieldToHubAngle()));

    // controller
    //     .leftBumper()
    //     .whileTrue(rollerSubsystem.intakeCommand())
    //     .whileTrue(agitatorSubsystem.intakeCommand());

    // controller
    //     .rightBumper()
    //     .whileTrue(indexerSubsystem.runCurrentCommand())
    //     .whileTrue(kickerSubsystem.indexCommand())
    //     .whileTrue(agitatorSubsystem.indexCommand());

    // controller.povDown().whileTrue(hoodSubsystem.runDebuggingVoltageDownCommand());
    // controller.povUp().whileTrue(hoodSubsystem.runDebuggingVoltageUpCommand());

    // controller.rightTrigger().whileTrue(rollerSubsystem.runUnjamCommand());

    // controller.povRight().whileTrue(flywheelSubsystem.runDebuggingVelocityCommand());

    // // Bug, Only Updated Hood Angle Once
    // controller
    //     .povLeft()
    //     .whileTrue(flywheelSubsystem.shootCommand())
    //     .whileTrue(hoodSubsystem.shootCommand());

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

  //     instance.start();
  //     Commands.runOnce(
  //             () -> {
  //               FuelSim.getInstance().clearFuel();
  //               FuelSim.getInstance().spawnStartingFuel();
  //             })
  //         .schedule();
  //   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
  public Command preLoadAutoCommand(){
    // Pose2d firstTargetPose2d = (DriverStation.getAlliance().get() == Alliance.Red) ? new Pose2d(15.205, 2.127, new Rotation2d()) : new Pose2d(1.154, 5.391, new Rotation2d(Math.PI)); 
    return Commands.sequence(
      // Commands.runOnce(
      //   () -> {
      //     new HolonomicAutoAlign(swerveSubsystem, new Pose2d(0,0, new Rotation2d()));
      //   }),
      
      Commands.parallel(flywheelSubsystem.runStaticVelocitCommand(), hoodSubsystem.runStaticAngleCommand()),
      new WaitCommand(4),
      Commands.parallel(
        DriveCommands.joystickDriveAtAngle(
                swerveSubsystem,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> shotCalculator.getFieldToHubAngle()),
        indexerSubsystem.runCurrentCommand(),
        agitatorSubsystem.indexCommand(),
        kickerSubsystem.indexCommand()
        )
        );
    
    // return Commands.runOnce(() -> {flywheelSubsystem.shootCommand();});

  }

  //   public Command depotAutoCommand(){
  //    Pose2d firstTargetPose2d = (DriverStation.getAlliance().get() == Alliance.Red) ? new Pose2d(15.205, 2.127, new Rotation2d()) : new Pose2d(1.154, 5.391, new Rotation2d(Math.PI)); 
  //    Pose2d secondTargetPose2d = (DriverStation.getAlliance().get() == Alliance.Red) ? new Pose2d(15.813, 2.088, new Rotation2d()) : new Pose2d(0.571, 5.918, new Rotation2d(Math.PI)); 
  //   return Commands.sequence(
  //     Commands.runOnce(
  //       () -> {
  //         new HolonomicAutoAlign(swerveSubsystem, firstTargetPose2d);
  //       }),
  //     Commands.sequence(
  //       pivotSubsystem.deployCommand(),
  //       rollerSubsystem.intakeCommand()
  //     ),
  //     Commands.runOnce(
  //       () -> {
  //         new HolonomicAutoAlign(swerveSubsystem, secondTargetPose2d);
  //     }),
      
  //     Commands.parallel(flywheelSubsystem.shootCommand(), hoodSubsystem.shootCommand()),
  //     Commands.parallel(
  //       DriveCommands.joystickDriveAtAngle(
  //               swerveSubsystem,
  //               () -> -controller.getLeftY(),
  //               () -> -controller.getLeftX(),
  //               () -> shotCalculator.getFieldToHubAngle()),
  //       indexerSubsystem.runCurrentCommand(),
  //       agitatorSubsystem.indexCommand(),
  //       kickerSubsystem.indexCommand()
  //       )
  //       );
    
  //   // return Commands.runOnce(() -> {flywheelSubsystem.shootCommand();});

  // }

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

  // public boolean isAllianceHandledAlready() {
  //   boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;

  //   Rotation2d currentHeading = swerveSubsystem.getRotation();

  //   if (isRed
  //       && ((currentHeading.getDegrees() > 170 && currentHeading.getDegrees() < 190)
  //           || (currentHeading.getDegrees() > -190 && currentHeading.getDegrees() < -170))) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  public boolean isAllianceHandledAlready() {
    if (!DriverStation.getAlliance().isPresent()) return false;
    if (lastAppliedAlliance == null) return false;
    return lastAppliedAlliance == DriverStation.getAlliance().get();
  }
  //   boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;

  //   Rotation2d currentHeading = swerveSubsystem.getRotation();

  //   if (isRed
  //       && ((currentHeading.getDegrees() > 170 && currentHeading.getDegrees() < 190)
  //           || (currentHeading.getDegrees() > -190 && currentHeading.getDegrees() < -170))) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  /**
   * Returns whether vision estimates should be clamped.
   *
   * @return true if vision estimates should be clamped. Enabled by default.
   */
  //   public Integer enableVisionClamp() {
  //     return clampVisionChooser.get();
  //   }
}
