package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.*;

public class HolonomicAutoAlign extends Command {
  private SwerveSubsystem driveSubsystem;
  private Pose2d targetPose;
  public Field2d targetfield = new Field2d();
  public static PathConstraints defaultPathfindingConstraints =
      new PathConstraints(2.0, 3.5, Units.degreesToRadians(540), Units.degreesToRadians(720));
  private final HolonomicDriveController holonomicDriveController;
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController rotController;
  public SwerveModuleState[] input;
  public ChassisSpeeds chassis;
  private double endingVelocity;


  public HolonomicAutoAlign(SwerveSubsystem driveSubsystem, Pose2d targetPose, double endingVelocity, boolean lowTolerance) {
    targetfield.setRobotPose(targetPose);
    SmartDashboard.putData("TargetField", targetfield);
    this.driveSubsystem = driveSubsystem;
    this.targetPose = targetPose;
    this.endingVelocity = endingVelocity;
    //TODO tune pid values for LeBobaJames
    xController = new PIDController(3.25, 0, 0.5); // Max Low Accuracy: 3.5, 0, 0.25
    yController = new PIDController(3.25, 0, 0.5); // Testing: 3.5, 0, 0.5

    rotController = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(5, 5));
    holonomicDriveController =
        new HolonomicDriveController(xController, yController, rotController);
    if(lowTolerance){
        //TODO make an reasonable low tolerance values
        holonomicDriveController.setTolerance(
            new Pose2d(new Translation2d(0.125, 0.125), Rotation2d.fromDegrees(5)));
    }else{
        holonomicDriveController.setTolerance(
            new Pose2d(new Translation2d(0.015, 0.015), Rotation2d.fromDegrees(1)));
    }

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("AutoAlign Status", true);
  }

  @Override
  public void execute() {
    chassis =
        holonomicDriveController.calculate(
            driveSubsystem.getPose(), targetPose, endingVelocity, targetPose.getRotation());

    // input = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassis);
    driveSubsystem.runVelocity(chassis);
    System.out.println("running");
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("AutoAlign Status", false);
    System.out.println("Ended AutoAlignCommand Early");
  }

  @Override
  public boolean isFinished() {
    return holonomicDriveController.atReference();
  }
}