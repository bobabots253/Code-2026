package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
  private final FlywheelSubsystem m_flywheel;
  private final HoodSubsystem m_hood;
  private final ShotCalculator m_calculator;

  //   private final shooterFuelSim shooterFuelSim;

  public ShooterSubsystem(
      FlywheelSubsystem flywheel,
      HoodSubsystem hood,
      ShotCalculator calculator,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.m_flywheel = flywheel;
    this.m_hood = hood;
    this.m_calculator = calculator;

    // shooterFuelSim =
    //     new shooterFuelSim(
    //         () ->
    //             new Pose3d(poseSupplier.get())
    //                 .transformBy(ShootOnTheFlyConstants.ROBOT_TO_SHOOTER_TRANSFORM),
    //         fieldSpeedsSupplier);

    // this.setDefaultCommand(
    //     shooterFuelSim.repeatedlyLaunchFuel(
    //         () -> {
    //           return Units.MetersPerSecond.of(calculator.getCorrectTargetVelocity());
    //         },
    //         () -> Units.Degrees.of(calculator.getCorrectedTargetAngle()),
    //         this));
  }

  public Command ShootOnTheFlyCommand() {
    return Commands.parallel(
        m_flywheel.run(() -> m_flywheel.runFlywheelRPM(m_calculator.getCorrectedTargetSpeedRPM())),
        m_hood.run(() -> m_hood.setGoalParams(m_calculator.getCorrectedTargetAngle(), 0.0)));
  }

  //   public Command simShootOnTheFlyCommand() {
  //     return shooterFuelSim.repeatedlyLaunchFuel(
  //         () -> {
  //           return Units.MetersPerSecond.of(m_calculator.getCorrectTargetVelocity());
  //         },
  //         () -> Units.Degrees.of(m_calculator.getCorrectedTargetAngle()),
  //         this);
  //   }

  //   @Override
  //   public void periodic() {
  //     shooterFuelSim.updateFuel(
  //         Units.MetersPerSecond.of(m_calculator.getCorrectTargetVelocity()),
  //         Units.Degrees.of(m_calculator.getCorrectedTargetAngle()));

  //     // Log the calculated targets for debugging
  //     Logger.recordOutput("ShooterSubsystem/TargetRPM",
  // m_calculator.getCorrectedTargetSpeedRPM());
  //     Logger.recordOutput(
  //         "ShooterSubsystem/TargetVelocityMPS", m_calculator.getCorrectTargetVelocity());
  //     Logger.recordOutput("ShooterSubsystem/TargetAngleDeg",
  // m_calculator.getCorrectedTargetAngle());
  //   }

  //   public boolean simAbleToIntake() {
  //     return shooterFuelSim.canIntake();
  //   }

  //   public void simIntake() {
  //     shooterFuelSim.intakeFuel();
  //   }

  // Broken
  //   public Command setToSafePosition(){
  //     return Commands.(
  //         this.run(() -> {
  //              m_hood.setGoalParams(45, 0);
  //         }), m_hood
  //     );
  //   }
}
