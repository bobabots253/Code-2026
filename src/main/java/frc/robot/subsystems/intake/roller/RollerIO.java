package frc.robot.subsystems.intake.roller;

import java.util.zip.Deflater;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public class rollerIOInputs {
        // public boolean rollerConnected = false;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;
    }
    
 public default void updateInputs(rollerIOInputs inputs) {}

 public default void setRollerOpenLoop(double speed) {}
}
