package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.BooleanSupplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends FullSubsystem {
  private static final double minAngle = Units.degreesToRadians(45);
  private static final double maxAngle = Units.degreesToRadians(85);

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  private Debouncer masterVortexConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private Alert motorDisconnectedAlert;

  @Setter private BooleanSupplier coastOverride = () -> false;

  private double goalAngle = 0.0; // Degrees
  private double goalVelocity = 0.0; // USe for Hard Zeroing

  private static double hoodOffset = HoodConstants.hoodOffsetDeg; // Note: DEGREES, not radians
  private boolean hoodZeroed = false;

  public HoodSubsystem(HoodIO io) {
    this.io = io;

    motorDisconnectedAlert = new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts()
            && !masterVortexConnectedDebouncer.calculate(inputs.masterMotorConnected));

    // Brake Mode when disabled
    if (DriverStation.isDisabled() || !hoodZeroed) {
      outputs.mode = HoodIOOutputMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = HoodIOOutputMode.COAST;
      }
    }
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isEnabled() && hoodZeroed) {
      outputs.positionRad =
          Units.degreesToRadians(MathUtil.clamp(goalAngle, minAngle, maxAngle) - hoodOffset);
      outputs.velocityRadsPerSec = goalVelocity;
      outputs.mode = HoodIOOutputMode.CLOSED_LOOP;

      // Log state
      Logger.recordOutput("Hood/Profile/GoalPositionRad", goalAngle);
      Logger.recordOutput("Hood/Profile/GoalVelocityRadPerSec", goalVelocity);
    }

    io.applyOutputs(outputs);
  }

  /**
   * Sets the goal parameters for the hood subsystem. When GoalParams are passed into
   * periodicAfterScheduler, the requested position and velocity will be clamped by minAngle and
   * maxAngle. Additional check, hoodZeroed. Input units in Degrees and Rad/Sec. Output units in
   * Degrees and Rad/Sec.
   *
   * @param angle
   * @param velocity
   */
  private void setGoalParams(double angle, double velocity) {
    goalAngle = angle;
    goalVelocity = velocity;
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleRads")
  public double getMeasuredAngleRad() {
    return inputs.positionRads + hoodOffset;
  }

  @AutoLogOutput
  public boolean atGoal() {
    return DriverStation.isEnabled()
        && hoodZeroed
        && Math.abs(getMeasuredAngleRad() - Units.degreesToRadians(goalAngle))
            <= Units.degreesToRadians(HoodConstants.toleranceDeg);
  }

  private void zero() {
    hoodOffset = Units.degreesToRadians(minAngle) - inputs.positionRads;
    hoodZeroed = true;
  }

  // public Command runTrackTargetCommand() {
  //   return runEnd(
  //       () ->
  //         setGoalParams(ShotCalculator.getInstance()...getHoodAngle()),
  //        this::stop);
  // }

  // public Command runFixedCommand(DoubleSupplier angle, DoubleSupplier velocity) {
  //   return runEnd(() -> setGoalParams(ShotCalculator.getInstance()...getHoodAngle(),
  // this::stop));
  // }

  public Command zeroCommand() {
    return runOnce(this::zero).ignoringDisable(true);
  }
}
