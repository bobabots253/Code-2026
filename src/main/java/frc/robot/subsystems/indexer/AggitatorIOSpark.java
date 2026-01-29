package frc.robot.subsystems.indexer;
import static frc.robot.subsystems.indexer.AggitatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;

public class AggitatorIOSpark implements AggitatorIO {
    private final SparkBase aggitatorSpark;
    private final Debouncer aggitatorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
public AggitatorIOSpark () {
        aggitatorSpark = new SparkMax(kAggitatorCANID, MotorType.kBrushless);
         var aggitatorConfig = new SparkMaxConfig();
        aggitatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kAggitatorCurrentLimit)
        .voltageCompensation(12.0);
        aggitatorConfig.signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        aggitatorSpark,
        5,
        () ->
            aggitatorSpark.configure(
                aggitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
}
public void updateInputs(AggitatorIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        aggitatorSpark,
        new DoubleSupplier[] {aggitatorSpark::getAppliedOutput, aggitatorSpark::getBusVoltage},
        (values) -> inputs.AggitatorAppliedVolts = values[0] * values[1]);
    ifOk(
        aggitatorSpark,
        aggitatorSpark::getOutputCurrent,
        (value) -> inputs.AggitatorCurrentAmps = value);
    inputs.AggitatorConnected = aggitatorDebouncer.calculate(!sparkStickyFault);
}
public void setPercentOutput(double value) {
}
}

