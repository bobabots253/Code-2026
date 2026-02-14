package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends FullSubsystem {
  private HoodIO hoodIO;
  private final Alert hoodDisconnected;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();
  private final Debouncer hoodDebouncer = new Debouncer(0.2, DebounceType.kFalling);
  private double hoodOffset = 0.0;
  private double goalAngle = 0.0;
  private double goalVelocity = 0.0;
  private Boolean hoodZeroed = false;
  public HoodSubsystem(HoodIO hoodIO) {
    
   

    this.hoodIO = hoodIO;
    hoodDisconnected = new Alert("Hood Motor Disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(inputs);
    Logger.processInputs("Hood / Inputs", inputs);

    hoodDisconnected.set(
        Robot.showHardwareAlerts() && !hoodDebouncer.calculate(inputs.hoodSparkConnected));
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Hood / Outputs", outputs.mode);
    hoodIO.applyOutputs(outputs);
    outputs.hoodSetPosRad = MathUtil.clamp(goalAngle, minAngleRad, maxAngleRad) - hoodOffset;
    outputs.hoodSetVelocityRad = goalVelocity;
  }

  public void zeroHood(){
    hoodOffset = minAngleRad - inputs.hoodPosRad;
    hoodZeroed = true;
  }

  public void setGoalParams(double angle, double velocity) {
    angle = goalAngle;
    velocity = goalVelocity;
  }

  public void runVoltage() {
    outputs.mode = HoodIOMode.VOLTAGE;
  }

  public void runClosedLoop() {
    outputs.mode = HoodIOMode.CLOSED_LOOP_CONTROL;
  }

public void runProfiled() {
    outputs.mode = HoodIOMode.PROFILED_CONTROL;
  }

   public void stop() {
    outputs.mode = HoodIOMode.BRAKE;
  }

  @AutoLogOutput(key = "Hood Position (Deg)")
  public double getHoodAngle(){
    return Units.radiansToDegrees(inputs.hoodPosRad + hoodOffset);
  }

  @AutoLogOutput
  public boolean hoodAtGoal() {
    return DriverStation.isEnabled()
        && hoodZeroed
        && Math.abs(getHoodAngle() - goalAngle)
            <= Units.degreesToRadians(toleranceDeg);

    }

  // public Command trackTarget() {
  //   return run (() -> )
  // }

  public Command staticTarget(DoubleSupplier angle, DoubleSupplier velocity) {
    return runEnd(()->setGoalParams(angle.getAsDouble(), velocity.getAsDouble()), this::stop);
  }

  public Command zero() {
    return runOnce(this::zeroHood).ignoringDisable(true);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
