package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

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
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;

public class FlywheelIOSpark implements FlywheelIO {
  // Declare REV motor hardware here
  private final SparkBase masterVortex;
  private final SparkBase followerVortex;

  // Declare REV encoders hardware here
  private final RelativeEncoder masterRelativeEncoder;
  private final RelativeEncoder followerRelativeEncoder;

  // Declare motor controllers
  private final SparkClosedLoopController masterVortexController;
  private final SparkClosedLoopController followerVortexController;

  public FlywheelIOSpark() {
    // Initialize REV motor hardware here
    masterVortex = new SparkFlex(sparkMasterFlywheelCanId, MotorType.kBrushless);
    followerVortex = new SparkFlex(sparkFollowerFlywheelCanId, MotorType.kBrushless);

    // Initialize REV encoders hardware here
    masterRelativeEncoder = masterVortex.getEncoder();
    followerRelativeEncoder = followerVortex.getEncoder();

    // Initialize motor controllers
    masterVortexController = masterVortex.getClosedLoopController();
    followerVortexController = followerVortex.getClosedLoopController();

    var masterVortexConfig = new SparkFlexConfig();
    masterVortexConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50) // Amps
        .voltageCompensation(12.0);
    masterVortexConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    masterVortexConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(sparkMasterFlyWheelkP, sparkMasterFlyWheelkI, sparkMasterFlyWheelkD);
    masterVortexConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (5))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        masterVortex,
        5,
        () ->
            masterVortex.configure(
                masterVortexConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    tryUntilOk(masterVortex, 5, () -> masterRelativeEncoder.setPosition(0.0));

    var followerVortexConfig = new SparkFlexConfig();
    followerVortexConfig
        .idleMode(IdleMode.kCoast)
        .follow(masterVortex)
        .smartCurrentLimit(50) // Amps
        .voltageCompensation(12.0);
    followerVortexConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    followerVortexConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(sparkMasterFlyWheelkP, sparkMasterFlyWheelkI, sparkMasterFlyWheelkD);
    followerVortexConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (5))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        followerVortex,
        5,
        () ->
            followerVortex.configure(
                followerVortexConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    tryUntilOk(followerVortex, 5, () -> followerRelativeEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.masterPositionRads =
        Units.rotationsToRadians(masterRelativeEncoder.getPosition()) / flywheelReductionRatio;
    inputs.masterVelocityRpm = masterRelativeEncoder.getVelocity() / flywheelReductionRatio;
    inputs.masterAppliedVolts = masterVortex.getAppliedOutput() * masterVortex.getBusVoltage();
    inputs.masterSupplyCurrentAmps = masterVortex.getOutputCurrent();

    inputs.followerPositionRads =
        Units.rotationsToRadians(followerRelativeEncoder.getPosition()) / flywheelReductionRatio;
    inputs.followerVelocityRpm = followerRelativeEncoder.getVelocity() / flywheelReductionRatio;
    inputs.followerAppliedVolts =
        followerVortex.getAppliedOutput() * followerVortex.getBusVoltage();
    inputs.followerSupplyCurrentAmps = followerVortex.getOutputCurrent();
  }

  @Override
  public void runVolts(double masterVolts) {
    masterVortex.setVoltage(masterVolts);
  }

  @Override
  public void runRPMSetpoint(double masterRPM) {
    masterVortexController.setSetpoint(
        masterRPM * flywheelReductionRatio,
        SparkBase.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        0,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    masterVortex.stopMotor();
    followerVortex.stopMotor();
  }
}
