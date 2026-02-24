package frc.robot.subsystems.kicker;

import static frc.robot.subsystems.kicker.KickerConstants.kA;
import static frc.robot.subsystems.kicker.KickerConstants.kS;
import static frc.robot.subsystems.kicker.KickerConstants.kV;
import static frc.robot.subsystems.kicker.KickerConstants.masterKickerEncoderPositionFactor;
import static frc.robot.subsystems.kicker.KickerConstants.masterKickerEncoderVelocityFactor;
import static frc.robot.subsystems.kicker.KickerConstants.maxAcceleration;
import static frc.robot.subsystems.kicker.KickerConstants.sparkMasterKickerkD;
import static frc.robot.subsystems.kicker.KickerConstants.sparkMasterKickerkI;
import static frc.robot.subsystems.kicker.KickerConstants.sparkMasterKickerkP;
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
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;

public class KickerIOSpark implements KickerIO {
  // Declare REV motor hardware here
  private final SparkBase masterNEO;

  // Declare REV encoders hardware here
  private final RelativeEncoder masterRelativeEncoder;

  // Declare motor controllers
  private final SparkClosedLoopController masterNEOController;

  // Declare WPILib Debouncer for Motor Disconnection Alerts here
  private final Debouncer masterNEODebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  private final SimpleMotorFeedforward ffCalculator = new SimpleMotorFeedforward(kS, kV, kA);
  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(maxAcceleration);

  // Hardware State Tracking
  private boolean wasCoasting = true;

  public KickerIOSpark() {
    // Initialize REV motor hardware here
    masterNEO = new SparkMax(KickerConstants.sparkMasterKickerCanId, MotorType.kBrushless);

    // Initialize REV encoders hardware here
    masterRelativeEncoder = masterNEO.getEncoder();

    // Initialize motor controllers here
    masterNEOController = masterNEO.getClosedLoopController();

    var masterNEOConfig = new SparkFlexConfig();
    masterNEOConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50) // Amps
        .voltageCompensation(12.0);
    masterNEOConfig
        .encoder
        .positionConversionFactor(masterKickerEncoderPositionFactor)
        .velocityConversionFactor(masterKickerEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    masterNEOConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(sparkMasterKickerkP, sparkMasterKickerkI, sparkMasterKickerkD);
    masterNEOConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(5)
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
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    // Update all the FlywheelIO inputs for the master motor
    // Use SparkStickyFaults to track motor connectivity (See SparkUtil for Implementation)
    sparkStickyFault = false;
    ifOk(
        masterNEO,
        masterRelativeEncoder::getPosition,
        (value) -> inputs.masterPositionRads = value);
    ifOk(
        masterNEO,
        masterRelativeEncoder::getVelocity,
        (value) -> inputs.masterVelocityRads = value);
    ifOk(
        masterNEO,
        new DoubleSupplier[] {masterNEO::getAppliedOutput, masterNEO::getBusVoltage},
        (values) -> inputs.masterAppliedVolts = values[0] * values[1]);
    ifOk(masterNEO, masterNEO::getAppliedOutput, (value) -> inputs.masterSupplyCurrentAmps = value);
    inputs.masterMotorConnected =
        masterNEODebouncer.calculate(!sparkStickyFault); // Force Connectivity Check
  }

  @Override
  public void applyOutputs(KickerIOOutputs outputs) {
    // Utilize the same code logic from the flywheel
    // See src\main\java\frc\robot\subsystems\shooter\flywheel\FlywheelIOSpark.java
    switch (outputs.mode) {
      case COAST:
      case VELOCITY_SETPOINT:
        masterNEO.stopMotor();
        break;
      case VOLTAGE:
        masterNEO.setVoltage(outputs.voltage);
    }
  }

  @Override
  public void runVolts(double masterVolts) {
    masterNEO.setVoltage(masterVolts);
  }

  @Override
  public void runVelocity(
      double velocity) { // Likely won't work due to FF implementation, but we can try this for funn
    masterNEOController.setSetpoint(velocity, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    masterNEO.stopMotor();
  }
}
