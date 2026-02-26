package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.maxAngleRad;
import static frc.robot.subsystems.shooter.hood.HoodConstants.minAngleRad;
import static frc.robot.subsystems.shooter.hood.HoodConstants.toleranceRad;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends FullSubsystem {
  private HoodIO io;
  private final Alert hoodDisconnected;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();
  private final Debouncer hoodDebouncer = new Debouncer(0.2, DebounceType.kFalling);
  private double hoodOffset = 0.0;
  private double goalAngle = 0.0;
  private double goalVelocity = 0.0;
  private Boolean hoodZeroed = false;

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0),

    PREPARE_HUB(() -> RobotState.getInstance().getCustomShotData().correctedTargetAngle()),

    SHOOT(() -> RobotState.getInstance().getCustomShotData().correctedTargetAngle()),

    JUGGLE(() -> HoodConstants.jugglingAngle),

    DEBUGGING(() -> HoodConstants.debuggingAngle);

    private final DoubleSupplier angleRads;

    private double getGoal() {
      return angleRads.getAsDouble();
    }
  }

  @AutoLogOutput private Goal currentGoal = Goal.IDLE;

  public HoodSubsystem(HoodIO io) {

    this.io = io;
    hoodDisconnected = new Alert("Hood Motor Disconnected", AlertType.kError);
    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Hood Idle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood / Inputs", inputs);

    hoodDisconnected.set(
        Robot.showHardwareAlerts() && !hoodDebouncer.calculate(inputs.masterNeoConnected));

    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    if (currentGoal == Goal.IDLE) {
      stop();
    } else {
      runAngular(currentGoal.getGoal());
    }
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Hood / Outputs", outputs.mode);
    io.applyOutputs(outputs);
    outputs.hoodSetPosRad = MathUtil.clamp(goalAngle, minAngleRad, maxAngleRad) - hoodOffset;
    outputs.hoodSetVelocityRad = goalVelocity;
  }

  private void setGoal(Goal desiredGoal) {
    this.currentGoal = desiredGoal;
  }

  private void runAngular(double angleRads) {
    outputs.mode = HoodIOMode.CLOSED_LOOP_CONTROL;
    outputs.hoodSetPosRad = angleRads;
  }

  public void setGoalParams(double angle, double velocity) {
    goalAngle = angle;
    goalVelocity = velocity;
  }

  public void setGoalParams(double angle) {
    goalAngle = angle;
  }

  public void modeVoltage() {
    outputs.mode = HoodIOMode.VOLTAGE;
  }

  public void modeClosedLoop() {
    outputs.mode = HoodIOMode.CLOSED_LOOP_CONTROL;
  }

  public void modeProfiled() {
    outputs.mode = HoodIOMode.PROFILED_CONTROL;
  }

  public void stop() {
    outputs.mode = HoodIOMode.BRAKE;
  }

  @AutoLogOutput(key = "Hood Position (Deg)")
  public double getHoodAngle() {
    return Units.radiansToDegrees(inputs.hoodPosRad + hoodOffset);
  }

  @AutoLogOutput(key = "Hood/AtGoal")
  public boolean hoodAtGoal() {
    return DriverStation.isEnabled()
        && hoodZeroed
        && Math.abs(getHoodAngle() - goalAngle) <= Units.degreesToRadians(toleranceRad);
  }

  // public Command trackTarget() {
  //    return run (() -> )
  // }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE)).withName("Hood Shoot");
  }

  public Command prepareHubCommand() {
    return startEnd(() -> setGoal(Goal.PREPARE_HUB), () -> setGoal(Goal.IDLE))
        .withName("Hood Prepare Hub");
  }

  public Command juggleCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE)).withName("Hood Juggle");
  }

  public Command debugCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING), () -> setGoal(Goal.IDLE)).withName("Hood Debug");
  }

  public Command runSetAngularCommand(DoubleSupplier angle) {
    return runEnd(() -> runAngular(angle.getAsDouble()), this::stop);
  }

  public Command staticTarget(DoubleSupplier angle) {
    return runEnd(() -> setGoalParams(angle.getAsDouble()), this::stop);
  }

  public Command zero() {
    return runOnce(() -> io.zeroHood(inputs)).ignoringDisable(true);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
