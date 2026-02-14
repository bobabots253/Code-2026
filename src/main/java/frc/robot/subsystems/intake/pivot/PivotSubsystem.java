package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.pivot.PivotIO.PivotIOOutputMode;
import frc.robot.subsystems.intake.pivot.PivotIO.PivotIOOutputs;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends FullSubsystem {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final PivotIOOutputs outputs = new PivotIOOutputs();

  private double kZeroSetpoint = 0.0;
  private double kTargetSetpoint = -15.0;

  private double goalAngle = 0.0;

  public PivotSubsystem(PivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isEnabled()) {
      outputs.positionRads = goalAngle;
      outputs.mode = PivotIOOutputMode.RUNNING;

      // Log state
      Logger.recordOutput("Pivot/Profile/GoalPositionRad", goalAngle);
    }

    io.applyOutputs(outputs);
  }

  public void setGoalParams(double goal) {
    goalAngle = goal;
  }

  public Command runTrackTargetCommand(double targetAngle) {
    return run(() -> io.lazyClosedLoop(targetAngle));
  }
}
