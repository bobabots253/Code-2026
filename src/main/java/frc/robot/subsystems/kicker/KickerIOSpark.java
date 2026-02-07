package frc.robot.subsystems.kicker;

import static frc.robot.subsystems.kicker.KickerConstants.kA;
import static frc.robot.subsystems.kicker.KickerConstants.kS;
import static frc.robot.subsystems.kicker.KickerConstants.kV;
import static frc.robot.subsystems.kicker.KickerConstants.masterKickerEncoderPositionFactor;
import static frc.robot.subsystems.kicker.KickerConstants.masterKickerEncoderVelocityFactor;
import static frc.robot.subsystems.kicker.KickerConstants.maxAcceleration;
import static frc.robot.subsystems.kicker.KickerConstants.sparkMasterKicker;
import static frc.robot.subsystems.kicker.KickerConstants.sparkMasterKickerkD;
import static frc.robot.subsystems.kicker.KickerConstants.sparkMasterKickerkI;
import static frc.robot.subsystems.kicker.KickerConstants.sparkMasterKickerkP;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class KickerIOSpark implements KickerIO {
  // Declare REV motor hardware here
  private final SparkBase masterVortex;

  // Declare REV encoders hardware here
  private final RelativeEncoder masterRelativeEncoder;

  // Declare motor controllers
  private final SparkClosedLoopController masterVortexController;

  private final SimpleMotorFeedforward ffCalculator = new SimpleMotorFeedforward(kS, kV, kA);
  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(maxAcceleration);

  // Hardware State Tracking
  private boolean wasCoasting = true;

  public KickerIOSpark() {
    // Initialize REV motor hardware here
    masterVortex = new SparkFlex(sparkMasterKicker, MotorType.kBrushless);

    // Initialize REV encoders hardware here
    masterRelativeEncoder = masterVortex.getEncoder();

    // Initialize motor controllers here
    masterVortexController = masterVortex.getClosedLoopController();

    var masterVortexConfig = new SparkFlexConfig();
    masterVortexConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50) // Amps
        .voltageCompensation(12.0);
    masterVortexConfig
        .encoder
        .positionConversionFactor(masterKickerEncoderPositionFactor)
        .velocityConversionFactor(masterKickerEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    masterVortexConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(sparkMasterKickerkP, sparkMasterKickerkI, sparkMasterKickerkD);
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
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.masterPositionRads = masterRelativeEncoder.getPosition();
    inputs.masterVelocityRads = masterRelativeEncoder.getVelocity();
    inputs.masterAppliedVolts = masterVortex.getAppliedOutput() * masterVortex.getBusVoltage();
    inputs.masterSupplyCurrentAmps = masterVortex.getOutputCurrent();
  }

  @Override
  public void applyOutputs(KickerIOOutputs outputs) {
    // Utilize the same code logic from the flywheel
    // See src\main\java\frc\robot\subsystems\shooter\flywheel\FlywheelIOSpark.java
    if (outputs.mode == KickerIOOutputMode.COAST) {
      masterVortex.stopMotor();
      wasCoasting = true;
      slewRateLimiter.reset(masterRelativeEncoder.getVelocity());
    } else {
      if (wasCoasting) {
        slewRateLimiter.reset(masterRelativeEncoder.getVelocity());
        wasCoasting = false;
      }
    }

    double profiledSetpoint = slewRateLimiter.calculate(outputs.velocityRadsPerSec);

    double accel =
        Math.abs(outputs.velocityRadsPerSec - profiledSetpoint) < 1.0
            ? 0.0
            : (outputs.velocityRadsPerSec > profiledSetpoint ? maxAcceleration : -maxAcceleration);

    double ffVolts = ffCalculator.calculate(profiledSetpoint, accel);

    masterVortexController.setSetpoint(
        profiledSetpoint,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        // kP should only be used to apply enough voltage to factor IRL discrepancies
        ffVolts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void runVolts(double masterVolts) {
    masterVortex.setVoltage(masterVolts);
  }

  @Override
  public void runVelocity(
      double velocity) { // Likely won't work due to FF implementation, but we can try this for funn
    masterVortexController.setSetpoint(velocity, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    masterVortex.stopMotor();
  }
}
