package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.FlywheelConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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

public class FlywheelIOSpark implements FlywheelIO {
  private final SparkBase flywheelSpark;
  private final SparkClosedLoopController flywheelController;
  private final Debouncer flywheelDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final RelativeEncoder flywheelEncoder;
  private final SparkBase kickerSpark;
  private final SparkClosedLoopController kickerController;
  private final Debouncer kickerDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final RelativeEncoder kickerEncoder;

  public FlywheelIOSpark() {
    flywheelSpark = new SparkMax(flywheelCanID, MotorType.kBrushless);
    flywheelEncoder = flywheelSpark.getEncoder();
    flywheelController = flywheelSpark.getClosedLoopController();
    kickerSpark = new SparkMax(kickerCanID, MotorType.kBrushless);
    kickerEncoder = kickerSpark.getEncoder();
    kickerController = kickerSpark.getClosedLoopController();
    var flywheelConfig = new SparkMaxConfig();
    flywheelConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(flywheelCurrentLimit)
        .voltageCompensation(12.0);
    flywheelConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(flywheelPIDMinOutput, flywheelPIDMaxOutput)
        .pid(flywheelVelocitykP, flywheelVelocitykI, flywheelVelocitykD);
    flywheelConfig
        .encoder
        .inverted(flywheelEncoderInverted)
        .positionConversionFactor(flywheelEncoderPositionFactor);
    flywheelConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        flywheelSpark,
        5,
        () ->
            flywheelSpark.configure(
                flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    var kickerConfig = new SparkMaxConfig();
    kickerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kickerCurrentLimit)
        .voltageCompensation(12.0);
    kickerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(kickerPIDMinOutput, kickerPIDMaxOutput)
        .pid(kickerVelocitykP, kickerVelocitykI, kickerVelocitykD);
    kickerConfig
        .encoder
        .inverted(kickerEncoderInverted)
        .positionConversionFactor(kickerEncoderPositionFactor);
    kickerConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        kickerSpark,
        5,
        () ->
            kickerSpark.configure(
                kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        flywheelSpark,
        new DoubleSupplier[] {flywheelSpark::getAppliedOutput, flywheelSpark::getBusVoltage},
        (values) -> inputs.flywheelAppliedVolts = values[0] * values[1]);

    ifOk(
        flywheelSpark,
        flywheelSpark::getOutputCurrent,
        (value) -> inputs.flywheelCurrentAmps = value);
    inputs.flywheelConnected = flywheelDebouncer.calculate(!sparkStickyFault);
    ifOk(flywheelSpark, flywheelEncoder::getVelocity, (value) -> inputs.flywheelVelocity = value);
    // kicker
    ifOk(
        kickerSpark,
        new DoubleSupplier[] {kickerSpark::getAppliedOutput, kickerSpark::getBusVoltage},
        (values) -> inputs.kickerAppliedVolts = values[0] * values[1]);

    ifOk(kickerSpark, kickerSpark::getOutputCurrent, (value) -> inputs.kickerCurrentAmps = value);
    inputs.kickerConnected = kickerDebouncer.calculate(!sparkStickyFault);
    ifOk(kickerSpark, kickerEncoder::getVelocity, (value) -> inputs.kickerVelocity = value);
  }

  @Override
  public void setRPM(double velocity) {
    double setPoint =
        MathUtil.inputModulus(
            flywheelEncoder.getPosition(), flywheelPIDMinOutput, flywheelPIDMaxOutput);
    flywheelController.setSetpoint(setPoint, ControlType.kVelocity);
  }

  @Override
  public void setKickerRPM(double velocity) {
    double setPoint =
        MathUtil.inputModulus(kickerEncoder.getVelocity(), kickerPIDMinOutput, kickerPIDMaxOutput);
    kickerController.setSetpoint(setPoint, ControlType.kVelocity);
  }
}
