package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.FeederConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class FeederIOSpark implements FeederIO {
  private final SparkBase feederSpark;
  private final SparkBaseConfig feederConfig;

  private final Debouncer feederConnectedDebounce =
      new Debouncer(feederDebounceTime, Debouncer.DebounceType.kFalling);

  public FeederIOSpark() {
    feederSpark = new SparkMax(feederCanId, MotorType.kBrushless);

    feederConfig = new SparkMaxConfig();
    feederConfig
        .idleMode(IdleMode.kBrake)
        .inverted(feederInverted)
        .smartCurrentLimit(feederCurrentLimit);

    tryUntilOk(
        feederSpark,
        5,
        () ->
            feederSpark.configure(
                feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(
        feederSpark,
        new DoubleSupplier[] {feederSpark::getAppliedOutput, feederSpark::getBusVoltage},
        (value) -> inputs.feederAppliedVolts = value[0] * value[1]);

    ifOk(feederSpark, feederSpark::getOutputCurrent, (value) -> inputs.feederCurrentAmps = value);

    inputs.feederConnected = feederConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setFeederOpenLoop(double output) {
  
  }
}
