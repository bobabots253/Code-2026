package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.intake.pivot.PivotConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PivotIOSpark implements PivotIO {
  /*
   * 1 spark base: generic rev motor
   * 1 relative encoder
   * 1 closed loop controller (pid controller)
   */

  // Declare REV motor hardware
  private final SparkBase pivotVortex;

  // Declare relative encoder
  private final RelativeEncoder pivotEncoder;

  // Declare closed loop controller
  private final SparkClosedLoopController pivotController;

  public PivotIOSpark() {
    pivotVortex = new SparkFlex(pivotCanID, MotorType.kBrushless);
    pivotEncoder = pivotVortex.getEncoder();
    pivotController = pivotVortex.getClosedLoopController();

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(pivotCurrentLimit);
    pivotConfig
        .encoder
        // .inverted(true)
        .positionConversionFactor(pivotPositionConversionFactor)
        .velocityConversionFactor(pivotVelocityConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(pivotkP, pivotkI, pivotkD);

    // add soft limits

    pivotConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        pivotVortex,
        5,
        () ->
            pivotVortex.configure(
                pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRads = pivotEncoder.getPosition();
    inputs.appliedVolts = pivotVortex.getAppliedOutput() * pivotVortex.getBusVoltage();
    inputs.supplyCurrentAmps = pivotVortex.getOutputCurrent();
  }

  @Override
  public void applyOutputs(PivotIOOutputs outputs) {
    double setpoint = outputs.positionRads;

    switch (outputs.mode) {
      case IDLE:
        pivotVortex.stopMotor();
        break;
      case RUNNING:
        pivotController.setSetpoint(
            setpoint, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        break;
      case BRAKE:
        pivotVortex.stopMotor();
        break;
    }
  }

  @Override
  public void setPercentageOpenLoop(double decimalPercentage) {
    pivotVortex.setVoltage(decimalPercentage);
  }

  @Override
  public void stop() {
    setPercentageOpenLoop(0.0);
  }

  @Override
  public void lazyClosedLoop(double target) {
    pivotController.setSetpoint(target, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
