package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj.Alert;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends FullSubsystem {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private Alert masterDisconnected;
  private Alert followerDisconnected;

  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;

    masterDisconnected = new Alert("Master flywheel disconnected!", Alert.AlertType.kWarning);
    followerDisconnected = new Alert("Follower flywheel disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    masterDisconnected.set(!inputs.masterMotorConnected);
    followerDisconnected.set(!inputs.followerMotorConnected);
  }

  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.masterSupplyCurrentAmps) > 50.0
        || Math.abs(inputs.followerSupplyCurrentAmps) > 50.0;
  }

  @Override
  public void periodicAfterScheduler() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodicAfterScheduler'");
  }
}
