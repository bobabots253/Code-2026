package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
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

  public HoodSubsystem(HoodIO io) {

    this.io = io;
    hoodDisconnected = new Alert("Hood Motor Disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood / Inputs", inputs);

    hoodDisconnected.set(
        Robot.showHardwareAlerts() && !hoodDebouncer.calculate(inputs.masterNeoConnected));
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Hood / Outputs", outputs.mode);
    io.applyOutputs(outputs);
    outputs.hoodSetPosRad = MathUtil.clamp(goalAngle, minAngleRad, maxAngleRad) - hoodOffset;
    outputs.hoodSetVelocityRad = goalVelocity;
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

  @AutoLogOutput
  public boolean hoodAtGoal() {
    return DriverStation.isEnabled()
        && hoodZeroed
        && Math.abs(getHoodAngle() - goalAngle) <= Units.degreesToRadians(toleranceRad);
  }

  // public Command trackTarget() {
  //   return run (() -> )
  // }

  public Command staticTarget(DoubleSupplier angle) {
    return runEnd(() -> setGoalParams(angle.getAsDouble()), this::stop);
  }

  public Command zero() {
    return runOnce(io.zeroHood(inputs)).ignoringDisable(true);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
