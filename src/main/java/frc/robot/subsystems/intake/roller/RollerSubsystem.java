package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.intake.roller.RollerIO.RollerIOOutputMode;
import frc.robot.subsystems.intake.roller.RollerIO.RollerIOOutputs;
import frc.robot.util.FullSubsystem;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class RollerSubsystem extends FullSubsystem {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private final RollerIOOutputs outputs = new RollerIOOutputs();
  private final Alert RollerDisconnectedAlert = new Alert("disconnected intake rollers",AlertType.kError);
  private final Debouncer RollerDebouncer = new Debouncer(0.2,DebounceType.kFalling);

@RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0),
    DEPLOYED(() -> RollerConstants.intakingVolts);
    // STOW(() -> RollerConstants.stowVolts), // Change if Necessary
    // JUGGLE(() -> RollerConstants.jugglingVolts),
    // DEBUGGING(() -> RollerConstants.debuggingVolts);

    private final DoubleSupplier voltage;

    /** Returns the current target voltage for this goal state. */
    private double getGoal() {
      return voltage.getAsDouble();
    }
  }
    @AutoLogOutput(key = "Roller/Goal")
  private Goal currentGoal = Goal.IDLE;



  public RollerSubsystem(RollerIO io) {
    this.io = io;

  }

  public void periodic() {
    //tells us what inputs are every 20ms
    io.updateInputs(inputs);
    Logger.processInputs("Intake roller log", inputs);
    RollerDisconnectedAlert.set(Robot.showHardwareAlerts() && RollerDebouncer.calculate(inputs.RollerConnected));
    
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Re-poll the supplier every loop to handle new updates
    if (currentGoal == Goal.IDLE) {
      stop();
    } else {
      runVoltage(currentGoal.getGoal());
    }
  }


@Override
  public void periodicAfterScheduler() {
  io.updateOutputs(outputs);
  Logger.recordOutput("Intake roller log", outputs.mode);
  }

  private void runVoltage(double volts) {
    outputs.mode = RollerIOOutputMode.VOLTAGE;
    outputs.rollerSpeed = volts;
  }

    @AutoLogOutput(key = "Roller/MeasuredVoltage")
  public double getVoltage() {
    return inputs.rollerAppliedVolts;
  }

    private void setGoal(Goal desiredGoal) {
    this.currentGoal = desiredGoal;
  }
  //lalalalalalalalalala (dont delete this)
  private void stop() {
    outputs.mode = RollerIOOutputMode.COAST;
    outputs.rollerSpeed = 0;
  }

  public Command intakeCommand() {
    return startEnd(() -> setGoal(Goal.DEPLOYED), () -> setGoal(Goal.IDLE))
        .withName("Roller Deploy");
  }

  public Command stopCommand() {
    return runOnce(this:: stop);
  }
//doubleSupplier = list of doubles 
  public Command setVoltage(DoubleSupplier volts) {
    return runEnd(() -> runVoltage(volts.getAsDouble()),this:: stop);
  }

  


}
