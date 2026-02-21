package frc.robot.subsystems.agitator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.agitator.AgitatorIO.AgitatorIOInputs;
import java.util.function.DoubleSupplier;

public class AgitatorIOSpark implements AgitatorIO {
  private final SparkBase masterNEO;

  private final Debouncer masterNEODebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  public AgitatorIOSpark() {
    masterNEO = new SparkMax(11, MotorType.kBrushless);
    SparkMaxConfig masterSparkMaxConfig = new SparkMaxConfig();
    masterSparkMaxConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
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
                PersistMode.kNoPersistParameters));
  }

  @Override
  public void updateInputs(AgitatorIOInputs inputs) {

    sparkStickyFault = false;
    ifOk(
        masterNEO,
        new DoubleSupplier[] {masterNEO::getAppliedOutput, masterNEO::getBusVoltage},
        (values) -> inputs.masterAppliedVolts = values[0] * values[1]);

    ifOk(masterNEO, masterNEO::getAppliedOutput, (value) -> inputs.masterSupplyCurrentAmps = value);
    inputs.masterMotorConnected = masterNEODebouncer.calculate(!sparkStickyFault);
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
