package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class FlywheelIOSpark implements FlywheelIO {
  private final SparkBase flywheelMasterVortex;
  private final SparkBase flywheelFollowerVortex;
  private final SparkClosedLoopController flywheelController;
  private final Debouncer flywheelDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final RelativeEncoder flywheelMasterEncoder;
  private final RelativeEncoder flywheelFollowerEncoder;

  public FlywheelIOSpark() {
    flywheelMasterVortex = new SparkFlex(flywheelMasterCanID, MotorType.kBrushless);
    flywheelFollowerVortex = new SparkFlex(flywheelFollowerCanID, MotorType.kBrushless);
    flywheelMasterEncoder = flywheelMasterVortex.getEncoder();
    flywheelFollowerEncoder = flywheelFollowerVortex.getEncoder();
    flywheelController = flywheelMasterVortex.getClosedLoopController();
    var flywheelMasterConfig = new SparkFlexConfig();
    flywheelMasterConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(flywheelCurrentLimit)
        .voltageCompensation(12.0);
    flywheelMasterConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(flywheelPIDMinOutput, flywheelPIDMaxOutput)
        .pid(flywheelVelocitykP, flywheelVelocitykI, flywheelVelocitykD);
    flywheelMasterConfig
        .encoder
        .inverted(flywheelEncoderInverted)
        .positionConversionFactor(flywheelEncoderPositionFactor);
    flywheelMasterConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        flywheelMasterVortex,
        5,
        () ->
            flywheelMasterVortex.configure(
                flywheelMasterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    var flywheelFollowerConfig = new SparkFlexConfig();
    flywheelFollowerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(flywheelCurrentLimit)
        .voltageCompensation(12.0)
        .follow(flywheelMasterVortex, true);
    flywheelFollowerConfig
        .encoder
        .inverted(flywheelEncoderInverted)
        .positionConversionFactor(flywheelEncoderPositionFactor);
    flywheelFollowerConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        flywheelFollowerVortex,
        5,
        () ->
            flywheelFollowerVortex.configure(
                flywheelFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        flywheelMasterVortex,
        new DoubleSupplier[] {
          flywheelMasterVortex::getAppliedOutput, flywheelMasterVortex::getBusVoltage
        },
        (values) -> inputs.flywheelMasterVolts = values[0] * values[1]);
    inputs.flywheelMasterCurrentAmps = flywheelMasterVortex.getOutputCurrent();
    inputs.flywheelMasterPosRad = flywheelMasterEncoder.getPosition();
    inputs.flywheelMasterVelocityRad = flywheelMasterEncoder.getVelocity();
    inputs.flywheelMasterConnected = flywheelDebouncer.calculate(!sparkStickyFault);

    ifOk(
        flywheelFollowerVortex,
        new DoubleSupplier[] {
          flywheelFollowerVortex::getAppliedOutput, flywheelFollowerVortex::getBusVoltage
        },
        (values) -> inputs.flywheelFollowerVolts = values[0] * values[1]);
    inputs.flywheelFollowerCurrentAmps = flywheelFollowerVortex.getOutputCurrent();
    inputs.flywheelFollowerPosRad = flywheelFollowerEncoder.getPosition();
    inputs.flywheelFollowerVelocityRad = flywheelFollowerEncoder.getVelocity();
    inputs.flywheelFollowerConnected = flywheelDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST:
        flywheelMasterVortex.stopMotor();
        break;
      case VOLTAGE:
        flywheelMasterVortex.set(1);
        break;
      case CLOSED_LOOP:
        flywheelController.setSetpoint(outputs.velocityRadsPerSec, ControlType.kVelocity);
        break;
    }
  }
}
