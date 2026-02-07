package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.sparkMasterFlyWheelkP;
import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;

// Note: CAD'ed Absolute Encoder is pointless
public class HoodIOSpark implements HoodIO {
  // Declare REV motor hardware here
  private final SparkBase masterVortex;

  // Declare REV encoders hardware here
  private final RelativeEncoder masterRelativeEncoder;

  // Declare motor controllers
  private final SparkClosedLoopController masterVortexController;

  public HoodIOSpark() {
    // Initialize REV motor hardware here
    masterVortex =
        new SparkFlex(HoodConstants.sparkMasterHoodCanId, SparkBase.MotorType.kBrushless);

    // Initialize REV encoders hardware here
    masterRelativeEncoder = masterVortex.getEncoder();

    // Initialize motor controllers here
    masterVortexController = masterVortex.getClosedLoopController();

    var masterVortexConfig = new SparkFlexConfig();
    masterVortexConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(35);
    masterVortexConfig
        .encoder
        .positionConversionFactor(masterPositionConversionFactor) // Rot to Rad // (2.0 * Math.PI) / kTotalReduction
        .velocityConversionFactor(masterVelocityConversionFactor)
        .inverted(false)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    masterVortexConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(sparkMasterHoodkP, sparkMasterHoodkI, sparkMasterHoodkD);
    masterVortexConfig.softLimit
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(minAngleRad)
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(maxAngleRad);
    masterVortexConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (20))
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

    // Autohome to a known position, if possible. If not, hope for the best
    masterRelativeEncoder.setPosition(minAngleRad);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Find a way to log master connection status
    inputs.positionRads = masterRelativeEncoder.getPosition(); // Rot to Rad conversion is done in config
    inputs.positionRadsPerSec = masterRelativeEncoder.getVelocity();
    inputs.appliedVoltage = masterVortex.getAppliedOutput() * masterVortex.getBusVoltage();
    inputs.supplyCurrentAmps = masterVortex.getOutputCurrent();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {

    // Safety check to prevent over/under extension
    // May need additional acceleration limits if the rack teeth start shearing
    double safeSetpoint = Math.max(minAngleRad, Math.min(maxAngleRad, outputs.positionRad));

    switch (outputs.mode) {
      case COAST:
        masterVortex.stopMotor();
        break;
      case BRAKE:
        masterVortex.stopMotor();
        break;
      case CLOSED_LOOP:
        masterVortexController.setSetpoint(
            safeSetpoint,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0);
        break;
    }
  }

  // Utility for ensuring correct direction and possibly for characterization
  // Use like <0.01 for testing if motor is going to be a V1.1
  @Override
  public void runOpenLoop(double decimalPercentage) {
    masterVortex.set(decimalPercentage);
  }
}
