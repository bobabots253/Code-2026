package frc.robot.subsystems.intake.roller;

import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;
// lalalalalalala

import edu.wpi.first.wpilibj2.command.Command;

public class RollerSubsystem extends FullSubsystem {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public RollerSubsystem(RollerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake roller log", inputs);
    
  }

  public Command setSpeed(double speed) {
    return runOnce(() -> io.setRollerOpenLoop(speed));
  }

  @Override
  public void periodicAfterScheduler() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodicAfterScheduler'");
  }
}
