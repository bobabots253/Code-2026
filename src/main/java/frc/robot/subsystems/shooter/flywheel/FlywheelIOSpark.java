package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.flywheelReductionRatio;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.followerFlywheelEncoderPositionFactor;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.followerFlywheelEncoderVelocityFactor;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kA;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kS;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kV;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.masterFlywheelEncoderPositionFactor;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.masterFlywheelEncoderVelocityFactor;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.maxAcceleration;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.sparkFollowerFlywheelCanId;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.sparkMasterFlyWheelkD;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.sparkMasterFlyWheelkI;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.sparkMasterFlyWheelkP;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.sparkMasterFlywheelCanId;
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

  private final SimpleMotorFeedforward ffCalculator = new SimpleMotorFeedforward(kS, kV, kA);
  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(maxAcceleration);

  private double lastSetpoint = 0.0;
  private boolean wasCoasting = true;

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
    masterVortexConfig
        .encoder
        .positionConversionFactor(masterFlywheelEncoderPositionFactor)
        .velocityConversionFactor(masterFlywheelEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
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
        .follow(masterVortex, true)
        .smartCurrentLimit(50) // Amps
        .voltageCompensation(12.0);
    followerVortexConfig
        .encoder.uvwMeasurementPeriod(10)
        .positionConversionFactor(followerFlywheelEncoderPositionFactor)
        .velocityConversionFactor(followerFlywheelEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
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
  public void applyOutputs(FlywheelIOOutputs outputs) {
    if (outputs.mode == FlywheelIOOutputMode.COAST) {
        masterVortex.stopMotor();
        wasCoasting = true;
    } else {
        if (wasCoasting) {
            slewRateLimiter.reset(masterVortex.getEncoder().getVelocity());
            wasCoasting = false;
        }
    }

    // Generate Simple Profiled Setpoint
    double profiledSetpoint = slewRateLimiter.calculate(outputs.velocityRadsPerSec);
    
    // Calculate acceleration for the kA term so motors don't flicker when the setpoint changes drastically
    double accel = Math.abs(outputs.velocityRadsPerSec - profiledSetpoint) < 1.0 
                     ? 0.0 : (outputs.velocityRadsPerSec > profiledSetpoint ? maxAcceleration : -maxAcceleration);
    double ffVolts = ffCalculator.calculate(profiledSetpoint, accel);

    // Set the setpoint with the calculated feedforward
    masterVortexController.setSetpoint(
          profiledSetpoint,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          ffVolts,
          SparkClosedLoopController.ArbFFUnits.kVoltage
      );
  }

  @Override
  public void runVolts(double masterVolts) {
    masterVortex.setVoltage(masterVolts);
  }

  @Override
  public void runVelocity(double velocity) { // Likely won't work due to FF implementation, but we can try this for funn
    masterVortexController.setSetpoint(
        velocity, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    masterVortex.stopMotor();
    followerVortex.stopMotor();
  }
}
