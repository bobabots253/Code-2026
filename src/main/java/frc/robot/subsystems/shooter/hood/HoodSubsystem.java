package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends FullSubsystem {
  private static final double minAngle = Units.degreesToRadians(45);
  private static final double maxAngle = Units.degreesToRadians(85);

  private static final double RadToDeg = 180.0 / Math.PI;
  private static final double DegToRad = Math.PI / 180.0;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg");

  static {
    kP.initDefault(3000);
    kD.initDefault(300);
    toleranceDeg.initDefault(1.0);
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);

  private BooleanSupplier coastOverride = () -> false;

  private double goalAngle = 0.0;
  private double goalVelocity = 0.0;

  private double positionDeg = 0.0;
  private double velocityDegPerSec = 0.0;

  private static double hoodOffset = 0.0;
  private boolean hoodZeroed = false;

  public HoodSubsystem(HoodIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    positionDeg = inputs.positionDegrees;
    velocityDegPerSec = inputs.velocityDegPerSec;

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.motorConnected));

    if (DriverStation.isDisabled() || !hoodZeroed) {
      outputs.mode = HoodIOOutputMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = HoodIOOutputMode.COAST;
      }
    }

    outputs.kP = kP.get();
    outputs.kD = kD.get();
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isEnabled() && hoodZeroed) {
      outputs.positionDegrees = MathUtil.clamp(goalAngle, minAngle, maxAngle) - hoodOffset;
      outputs.mode = HoodIOOutputMode.CLOSED_LOOP;
      Logger.recordOutput("Hood/Profile/GoalPositionDeg", goalAngle);
    }
    io.applyOutputs(outputs);
  }

  public void setGoalParams(double angle, double velocity) {
    goalAngle = angle;
    goalVelocity = velocity;
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleDeg")
  public double getMeasuredAngleDeg() {
    return inputs.positionDegrees + hoodOffset;
  }

  @AutoLogOutput
  public boolean atGoal() {
    return DriverStation.isEnabled()
        && hoodZeroed
        && Math.abs(getMeasuredAngleDeg() - goalAngle)
            <= Units.degreesToRadians(toleranceDeg.get());
  }

  private void zero() {
    hoodOffset = minAngle - inputs.positionDegrees;
    hoodZeroed = true;
  }
}
