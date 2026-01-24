package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelIOInputs {
        public boolean flywheelConnected = false;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;
        public double flywheelVelocity = 0.0;
    }

    public default void updateInputs(FlywheelIOInputs inputs){}

    public default void setRPM(){}

}
