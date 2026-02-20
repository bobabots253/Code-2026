package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  public record ShootingParameters(double speed, double angle, boolean readyToShoot) {}

  private ShootingParameters latestShotData = new ShootingParameters(0.0, 0, false);

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public void setShootingParameters(ShootingParameters data) {
    this.latestShotData = data;
  }

  @AutoLogOutput(key = "RobotState/ShootingParameters")
  public ShootingParameters getCustomShotData() {
    return latestShotData;
  }
}
