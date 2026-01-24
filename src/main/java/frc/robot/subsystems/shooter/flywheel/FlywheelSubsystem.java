package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends FullSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final double RPMToRadPerSec = 60.0 / (2.0 * Math.PI);
  private final double RadPerSecToRPM = 2.0 * Math.PI / 60.0;

  // Cached RPM Value
  private double velocityRPM = 0.0;

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollowerConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert followerDisconnected;

  private static final LoggedTunableNumber torqueCurrentControlToleranceRPM =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlToleranceRPM", 50.0);
  private static final LoggedTunableNumber torqueCurrentControlDebounce =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlDebounce", 0.025);
  private static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheel/AtGoalDebounce", 0.2);

  private Debouncer torqueCurrentDebouncer =
      new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
  private boolean lastTorqueCurrentControl = false;
  @AutoLogOutput private long shotCount = 0;

  @AutoLogOutput private boolean atGoal = false;

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;

    disconnected = new Alert("Flywheel motor disconnected!", Alert.AlertType.kWarning);
    followerDisconnected =
        new Alert("Flywheel follower motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    velocityRPM = inputs.velocityRadsPerSec * RadPerSecToRPM;

    if (torqueCurrentControlDebounce.hasChanged(hashCode())) {
      torqueCurrentDebouncer =
          new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
    }
    if (atGoalDebounce.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));
    followerDisconnected.set(
        Robot.showHardwareAlerts()
            && !motorFollowerConnectedDebouncer.calculate(inputs.followerConnected));
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    io.setFlywheelRPM(outputs);
  }

  private void runFlywheelRPM(double setpointRPM) {
    boolean inTolerance =
        Math.abs(velocityRPM - setpointRPM) <= torqueCurrentControlToleranceRPM.get();
    boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
    atGoal = atGoalDebouncer.calculate(inTolerance);

    if (!torqueCurrentControl && lastTorqueCurrentControl) {
      shotCount++;
    }
    lastTorqueCurrentControl = torqueCurrentControl;

    outputs.mode =
        torqueCurrentControl
            ? FlywheelIOOutputMode.TORQUE_CURRENT_BANG_BANG
            : FlywheelIOOutputMode.DUTY_CYCLE_BANG_BANG;
    outputs.velocityRadsPerSec = setpointRPM * RadPerSecToRPM;
    Logger.recordOutput("Flywheel/VelocityRPM", velocityRPM);
  }

  // Stop the flywheel, Sets mode to coast
  private void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
    atGoal = false;
  }

  public double getVelocityRadsPerSec() {
    return inputs.velocityRadsPerSec;
  }

  public Command runStaticSetpointCommand(DoubleSupplier RPM) {
    return runEnd(() -> runFlywheelRPM(RPM.getAsDouble()), this::stop);
  }
}
