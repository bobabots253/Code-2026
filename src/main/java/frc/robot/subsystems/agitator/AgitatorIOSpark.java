package frc.robot.subsystems.agitator;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class AgitatorIOSpark implements AgitatorIO {
  // Declare REV motor hardware here
  private final SparkBase masterNEO;

  // Declare WPILib Debouncer for Motor Disconnection Alerts here
  private final Debouncer masterNEODebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  public AgitatorIOSpark() {
    // Initialize REV motor hardware here
    masterNEO = new SparkMax(AgitatorConstants.sparkMasterAgitatorCanId, MotorType.kBrushless);
    SparkMaxConfig masterSparkMaxConfig = new SparkMaxConfig();
    masterSparkMaxConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AgitatorConstants.masterCurrentLimit)
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
  public void updateInputs(AgitatorIOInputs inputs) {
    // Update all the Agitator inputs for the master motor
    // Use SparkStickyFaults to track motor connectivity (See SparkUtil for Implementation)
    sparkStickyFault = false;
    ifOk(
        masterNEO,
        new DoubleSupplier[] {masterNEO::getAppliedOutput, masterNEO::getBusVoltage},
        (values) -> inputs.masterAppliedVolts = values[0] * values[1]);

    ifOk(masterNEO, masterNEO::getOutputCurrent, (value) -> inputs.masterSupplyCurrentAmps = value);
    
    ifOk(masterNEO, masterNEO::getMotorTemperature, (value) -> inputs.masterTempCelsius = value);
  }

  @Override
  public void applyOutputs(AgitatorIOOutputs outputs) {

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
}
