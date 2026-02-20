package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

public class HoodIOSpark implements HoodIO {
  // masterNeo is a big NEO
  private final SparkBase masterNeo;
  private final RelativeEncoder hoodEncoder;
  private final SparkClosedLoopController hoodController;
  private final ProfiledPIDController profiledHoodController;
  private final SimpleMotorFeedforward hoodFF =
      new SimpleMotorFeedforward(sparkHoodProfiledkS, sparkHoodProfiledkV);
  private final Debouncer hoodDebounce;

  public HoodIOSpark() {
    hoodDebounce = new Debouncer(0.5, DebounceType.kFalling);
    masterNeo = new SparkMax(sparkMasterHoodCanId, MotorType.kBrushless);
    hoodEncoder = masterNeo.getEncoder();
    hoodController = masterNeo.getClosedLoopController();

    profiledHoodController =
        new ProfiledPIDController(
            sparkHoodProfiledkP,
            sparkHoodProfiledkI,
            sparkHoodProfiledkD,
            new TrapezoidProfile.Constraints(sparkHoodMaxAccel, sparkHoodMaxVelocity),
            0.02);

    var hoodConfig = new SparkMaxConfig();
    hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(hoodCurrentLimit);
    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(minAngleRad, maxAngleRad)
        .pid(sparkHoodkP, sparkHoodkI, sparkHoodkD);
    hoodConfig
        .encoder
        .positionConversionFactor(masterPositionConversionFactor)
        .velocityConversionFactor(masterVelocityConversionFactor)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
    hoodConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(5)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(5)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        masterNeo,
        5,
        () ->
            masterNeo.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(HoodIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(
        masterNeo,
        new DoubleSupplier[] {masterNeo::getAppliedOutput, masterNeo::getBusVoltage},
        (values) -> inputs.hoodVolts = values[0] * values[1]);
    ifOk(masterNeo, masterNeo::getOutputCurrent, (value) -> inputs.hoodCurrentAmps = value);
    inputs.masterNeoConnected = hoodDebounce.calculate(!sparkStickyFault);
    inputs.hoodPosRad = hoodEncoder.getPosition();
    inputs.hoodVelocityRad = hoodEncoder.getVelocity();
  }

  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE:
        masterNeo.stopMotor();
        break;
      case VOLTAGE:
        setPercentVoltage(outputs.hoodDecimalPercentOutput);
        break;
      case CLOSED_LOOP_CONTROL:
        setClosedLoopControl(outputs);
        break;
      case PROFILED_CONTROL:
        setProfiledControl(outputs);
        break;
    }
  }

  @Override
  public void setPercentVoltage(double decimalPercent) {
    masterNeo.set(decimalPercent);
  }

  @Override
  public void setClosedLoopControl(HoodIOOutputs outputs) {
    hoodController.setSetpoint(outputs.hoodSetPosRad, ControlType.kPosition);
  }

  @Override
  public void setProfiledControl(HoodIOOutputs outputs) {
    masterNeo.setVoltage(
        profiledHoodController.calculate(
            (hoodEncoder.getPosition() - hoodOffset),
            outputs.hoodSetPosRad )+ hoodFF.calculate(outputs.hoodSetVelocityRad));
  }
@Override
    public void zeroHood(HoodIOInputs inputs) {
    hoodOffset = minAngleRad - inputs.hoodPosRad;
    hoodEncoder.setPosition(0.0);
  }
}
