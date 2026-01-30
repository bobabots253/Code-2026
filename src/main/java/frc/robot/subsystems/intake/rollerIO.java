package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;


public interface rollerIO {
    @AutoLog
    public class rollerIOInputs{

        public boolean rollerConnected = false;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;

    }
    public default void updateInputs(rollerIOInputs inputs) {}

  public default void setrollerOpenLoop() {}
} 
    

