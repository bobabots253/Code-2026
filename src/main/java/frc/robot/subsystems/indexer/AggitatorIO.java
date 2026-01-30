package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;


public interface AggitatorIO {
    @AutoLog
    public class AggitatorIOInputs{

        public boolean AggitatorConnected = false;
        public double AggitatorAppliedVolts = 0.0;
        public double AggitatorCurrentAmps = 0.0;

    }
    public default void updateInputs(AggitatorIOInputs inputs) {}

  public default void setPercentOutput(double SetAngle) {}
} 
