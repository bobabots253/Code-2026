package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.HoodConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class HoodIOSpark implements HoodIO {
  private final SparkBase hoodSpark;
  private final SparkClosedLoopController hoodController;
  private final AbsoluteEncoder hoodEncoder;
  private final Debouncer hoodConnectedDebounce = new Debouncer(0.5);

  public HoodIOSpark() {
    // frontIntakeBeamBreak = new DigitalInput(frontBeamBreakSensor);
    // backIntakeBeamBreak = new DigitalInput(backBeamBreakSensor);
    hoodSpark = new SparkMax(hoodCanID, MotorType.kBrushless);
    hoodEncoder = hoodSpark.getAbsoluteEncoder();
    hoodController = hoodSpark.getClosedLoopController();
    var hoodConfig = new SparkMaxConfig();
    hoodConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(hoodCurrentLimit)
        .voltageCompensation(12.0);
    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(hoodkPIDMaxInput, hoodkPIDMaxInput)
        .pid(hoodkP, 0.0, hoodkD);
    hoodConfig
        .absoluteEncoder
        .inverted(hoodEncoderInverted)
        .positionConversionFactor(hoodEncoderPositionFactor)
        .averageDepth(2);
    hoodConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        hoodSpark,
        5,
        () ->
            hoodSpark.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(
        hoodSpark,
        new DoubleSupplier[] {hoodSpark::getAppliedOutput, hoodSpark::getBusVoltage},
        (values) -> inputs.hoodAppliedVolts = values[0] * values[1]);
    ifOk(hoodSpark, hoodSpark::getOutputCurrent, (value) -> inputs.hoodCurrentAmps = value);
    inputs.hoodConnected = hoodConnectedDebounce.calculate(!sparkStickyFault);
    ifOk(hoodSpark, hoodEncoder::getPosition, (value) -> inputs.hoodPosition = value);
  }

  public void setHoodPos(double angle) {
    double setPoint =
        MathUtil.inputModulus(hoodEncoder.getPosition(), hoodkPIDMinInput, hoodkPIDMaxInput);
    hoodController.setSetpoint(setPoint, ControlType.kPosition);
  }
}
