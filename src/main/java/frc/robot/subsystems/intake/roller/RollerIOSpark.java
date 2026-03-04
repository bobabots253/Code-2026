package frc.robot.subsystems.intake.roller;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class RollerIOSpark implements RollerIO {
  // Declare REV motor hardware here
  private final SparkBase masterNEO;

  // Declare WPILib Debouncer for Motor Disconnection Alerts here
  private final Debouncer masterNEODebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  public RollerIOSpark() {
    // Initialize REV motor hardware here
    masterNEO = new SparkMax(RollerConstants.sparkMasterRollerCanId, MotorType.kBrushless);
    SparkMaxConfig masterSparkMaxConfig = new SparkMaxConfig();
    masterSparkMaxConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(RollerConstants.masterCurrentLimit)
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        masterNEO,
        5,
        () ->
            masterNEO.configure(
                masterSparkMaxConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {

    // Update all the RollerIO inputs for the master motor
    // Use SparkStickyFaults to track motor connectivity (See SparkUtil for Implementation)
    sparkStickyFault = false;
    ifOk(
        masterNEO,
        new DoubleSupplier[] {masterNEO::getAppliedOutput, masterNEO::getBusVoltage},
        (values) -> inputs.masterAppliedVolts = values[0] * values[1]);
    ifOk(masterNEO, masterNEO::getOutputCurrent, (value) -> inputs.masterSupplyCurrentAmps = value);
    inputs.masterMotorConnected =
        masterNEODebouncer.calculate(!sparkStickyFault); // Force Connectivity Check
    inputs.masterTempCelsius = masterNEO.getMotorTemperature();
    ifOk(masterNEO, masterNEO::getMotorTemperature, (value) -> inputs.masterTempCelsius = value);
  }

  @Override
  public void applyOutputs(RollerIOOutputs outputs) {

    double setpoint = outputs.voltage;

    switch (outputs.mode) {
      case BRAKE:
        masterNEO.stopMotor();
        break;
      case VOLTAGE:
        masterNEO.setVoltage(setpoint);
        break;
    }
  }

  @Override
  public void simpleVoltage(double volts) {
    masterNEO.setVoltage(volts);
  }
}
