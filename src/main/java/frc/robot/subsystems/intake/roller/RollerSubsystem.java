package frc.robot.subsystems.intake.roller;

import java.util.logging.Logger;

import frc.robot.subsystems.intake.rollerIOInputsAutoLogged;
import frc.robot.util.FullSubsystem;

// lalalalalalala
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
    public void setSpeed(double speed) {
        io.setRollerOpenLoop(speed);
    }

    @Override
    public void periodicAfterScheduler() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodicAfterScheduler'");
    }
}
