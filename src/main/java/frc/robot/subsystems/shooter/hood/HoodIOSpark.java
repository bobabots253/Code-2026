package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

// Note: CAD'ed Absolute Encoder is pointless
public class HoodIOSpark implements HoodIO {
  // Declare REV motor hardware here
  private final SparkBase masterNEO;

  // Declare REV encoders hardware here
  private final RelativeEncoder masterRelativeEncoder;

  // Declare motor controllers
  private final SparkClosedLoopController masterNEOController;

  // Declare WPILib Debouncer for Motor Disconnection Alerts here
  private final Debouncer masterNEODebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  public HoodIOSpark() {
    // Initialize REV motor hardware here
    masterNEO = new SparkMax(HoodConstants.sparkMasterHoodCanId, SparkBase.MotorType.kBrushless);

    // Initialize REV encoders hardware here
    masterRelativeEncoder = masterNEO.getEncoder();

    // Initialize motor controllers here
    masterNEOController = masterNEO.getClosedLoopController();

    var masterNEOConfig = new SparkMaxConfig();
    masterNEOConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(35); // Current Limit reduced for safety
    masterNEOConfig
        .encoder
        .positionConversionFactor(masterPositionConversionFactor) // THE USER MUST GET THIS RIGHT
        .velocityConversionFactor(masterVelocityConversionFactor) // Only used for logging
        .uvwMeasurementPeriod(
            10) // Measurement = Postion / deltaTime, this edits deltaTime to parameter
        .uvwAverageDepth(2); // Does not affect positional control
    masterNEOConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(sparkMasterHoodkP, sparkMasterHoodkI, sparkMasterHoodkD);
    masterNEOConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        masterNEO,
        5,
        () ->
            masterNEO.configure(
                masterNEOConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Assuming you never call reset position when you aren't supposed to
    // IF you know that we auto-zero on enable, then implement setPosition call to reset pos.
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Update all the HoodIO inputs for the master motor
    // Use SparkStickyFaults to track motor connectivity (See SparkUtil for Implementation)
    sparkStickyFault = false;
    ifOk(
        masterNEO,
        masterRelativeEncoder::getPosition,
        (value) -> inputs.masterPositionRads = value);
    ifOk(
        masterNEO,
        new DoubleSupplier[] {masterNEO::getAppliedOutput, masterNEO::getBusVoltage},
        (values) -> inputs.masterAppliedVolts = values[0] * values[1]);
    ifOk(masterNEO, masterNEO::getOutputCurrent, (value) -> inputs.masterSupplyCurrentAmps = value);
    inputs.masterMotorConnected =
        masterNEODebouncer.calculate(!sparkStickyFault); // Force Connectivity Check
    ifOk(masterNEO, masterNEO::getMotorTemperature, (value) -> inputs.masterTempCelsius = value);
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {

    // Safety check to prevent over/under extension
    // THE USER MUST GET THIS RIGHT
    double safeSetpoint = Math.max(minAngleRad, Math.min(maxAngleRad, outputs.positionRad));

    switch (outputs.mode) {
      case BRAKE:
        masterNEO.stopMotor(); // Internal REV API calls "set(0);"
        break;
      case CLOSED_LOOP:
        masterNEOController.setSetpoint(
            // kSlot0 is the default setting slot called in the config for pid
            safeSetpoint, SparkBase.ControlType.kPosition);
        break;
      case VOLTAGE:
        masterNEO.setVoltage(outputs.voltage);
    }
  }

  /**
   * Utility for ensuring correct direction and possibly for characterization. Use like <0.01 for
   * testing if motor is going to be a V1.1. Preferably do NOT use. Instead use the state based
   * system found in HoodSubsystem and edit the corresponding constructor arguement
   */
  @Override
  public void runOpenLoop(double decimalPercentage) {
    masterNEO.set(decimalPercentage);
  }
}
