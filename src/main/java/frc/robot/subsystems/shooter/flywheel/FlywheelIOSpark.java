package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.followerFlywheelEncoderPositionFactor;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.followerFlywheelEncoderVelocityFactor;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kA;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kAntilag;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kS;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kV;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.masterFlywheelEncoderPositionFactor;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.masterFlywheelEncoderVelocityFactor;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.sparkFollowerFlywheelCanId;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.sparkMasterFlywheelCanId;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Flywheel Hardware implementation using SparkFlex Current System is designed to mimics
 * VelocityTorqueCurrentFOC from PhoenixLib by using a continously computed Feedforward voltage.
 */
public class FlywheelIOSpark implements FlywheelIO {
  // Declare REV motor hardware here
  private final SparkBase masterVortex;
  private final SparkBase followerVortex;

  // Declare REV encoders hardware here
  private final RelativeEncoder masterRelativeEncoder;
  private final RelativeEncoder followerRelativeEncoder;

  // Declare motor controllers
  private final SparkClosedLoopController masterVortexController;

  //
  private final SimpleMotorFeedforward ffcalculator = new SimpleMotorFeedforward(kS, kV, kA);

  // Declare WPILib Debouncer for Motor Disconnection Alerts here
  private final Debouncer masterVortexDebouncer =
      new Debouncer(0.25, Debouncer.DebounceType.kFalling);
  private final Debouncer followerVortexDebouncer =
      new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  // Internal Flywheel Control State Tracking
  private enum FlywheelPhase {
    STARTUP,
    IDLE,
    BALL,
    RECOVERY
  }

  private FlywheelPhase currentPhase = FlywheelPhase.STARTUP;

  public FlywheelIOSpark() {
    // Initialize REV motor hardware here
    masterVortex = new SparkFlex(sparkMasterFlywheelCanId, MotorType.kBrushless);
    followerVortex = new SparkFlex(sparkFollowerFlywheelCanId, MotorType.kBrushless);

    // Initialize REV encoders hardware here
    masterRelativeEncoder = masterVortex.getEncoder();
    followerRelativeEncoder = followerVortex.getEncoder();

    // Initialize motor controllers
    masterVortexController = masterVortex.getClosedLoopController();

    var masterVortexConfig = new SparkFlexConfig();
    masterVortexConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50); // Amps
    // .voltageCompensation(12.0); // Try Again
    masterVortexConfig
        .encoder
        .positionConversionFactor(masterFlywheelEncoderPositionFactor) // Only used for logging
        .velocityConversionFactor(
            masterFlywheelEncoderVelocityFactor) // THE USER MUST GET THIS RIGHT
        .uvwMeasurementPeriod(
            10) // Measurement = Postion / deltaTime, this edits deltaTime to parameter
        .uvwAverageDepth(2); // Does not affect positional control
    masterVortexConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0, 0);
    masterVortexConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.03302, 0.0, 0.0, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(
            Units.rotationsPerMinuteToRadiansPerSecond(100), ClosedLoopSlot.kSlot1);
    masterVortexConfig.closedLoop.feedForward.sva(kS, kV, kA, ClosedLoopSlot.kSlot1);
    // Since Current Control (CC) uses internal motor measurements,
    // does this even affect
    // .p(sparkMasterFlyWheelkP, ClosedLoopSlot.kSlot0);
    // .d(sparkMasterFlyWheelkD, ClosedLoopSlot.kSlot0) // Need kD?
    // .allowedClosedLoopError(14, ClosedLoopSlot.kSlot0) // Respects CC
    // .maxOutput(1, ClosedLoopSlot.kSlot0)
    // .minOutput((-1), ClosedLoopSlot.kSlot0);
    masterVortexConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(10)
        .isAtSetpointAlwaysOn(true)
        .isAtSetpointPeriodMs(10)
        .setSetpointAlwaysOn(true)
        .setpointPeriodMs(10)
        .appliedOutputPeriodMs(10)
        .busVoltagePeriodMs(10)
        .outputCurrentPeriodMs(10);
    tryUntilOk(
        masterVortex,
        5,
        () ->
            masterVortex.configure(
                masterVortexConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    var followerVortexConfig = new SparkFlexConfig();
    followerVortexConfig
        .idleMode(IdleMode.kCoast)
        .follow(masterVortex, true) // Confirm this inversion before running full speed
        .smartCurrentLimit(50); // Amps, Max Out reasonable value for most power
    // .voltageCompensation(12.0); // Try Again
    followerVortexConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .positionConversionFactor(followerFlywheelEncoderPositionFactor)
        .velocityConversionFactor(followerFlywheelEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    followerVortexConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(10)
        .isAtSetpointAlwaysOn(true)
        .isAtSetpointPeriodMs(10)
        .setSetpointAlwaysOn(true)
        .setpointPeriodMs(10)
        .appliedOutputPeriodMs(10)
        .busVoltagePeriodMs(10)
        .outputCurrentPeriodMs(10);
    tryUntilOk(
        followerVortex,
        5,
        () ->
            followerVortex.configure(
                followerVortexConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    // Update all the FlywheelIO inputs for the master motor
    // Use SparkStickyFaults to track motor connectivity (See SparkUtil for Implementation)
    sparkStickyFault = false;
    ifOk(
        masterVortex,
        masterRelativeEncoder::getVelocity,
        (value) -> inputs.masterVelocityRads = value);
    ifOk(
        masterVortex,
        new DoubleSupplier[] {masterVortex::getAppliedOutput, masterVortex::getBusVoltage},
        (values) -> inputs.masterAppliedVolts = values[0] * values[1]);
    ifOk(
        masterVortex,
        masterVortex::getOutputCurrent,
        (value) -> inputs.masterSupplyCurrentAmps = value);
    inputs.masterMotorConnected =
        masterVortexDebouncer.calculate(!sparkStickyFault); // Force Connectivity Check
    ifOk(
        masterVortex,
        masterVortex::getMotorTemperature,
        (value) -> inputs.masterTempCelsius = value);

    // Update all the FlywheelIO inputs for the follower motor
    // Use SparkStickyFaults to track motor connectivity (See SparkUtil for Implementation)
    sparkStickyFault = false;
    ifOk(
        followerVortex,
        followerRelativeEncoder::getVelocity,
        (value) -> inputs.followerVelocityRads = value);
    ifOk(
        followerVortex,
        new DoubleSupplier[] {followerVortex::getAppliedOutput, followerVortex::getBusVoltage},
        (values) -> inputs.followerAppliedVolts = values[0] * values[1]);
    ifOk(
        followerVortex,
        followerVortex::getOutputCurrent,
        (value) -> inputs.followerSupplyCurrentAmps = value);
    inputs.followerMotorConnected =
        followerVortexDebouncer.calculate(!sparkStickyFault); // Force Connectivity Check
    ifOk(
        followerVortex,
        followerVortex::getMotorTemperature,
        (value) -> inputs.masterTempCelsius = value);
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    // Better Implementation of Bang-Bang and TorqueCurrentFOC cuz REV API ;(
    // Strucutre:
    // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272
    // TO-DO: Clean-Up this Code Section and add Acceleration Logging
    // TO-DO: Possibly Initiate Ball Sub-State using the Kicker Subsystem
    switch (outputs.mode) {
      case COAST:
        masterVortex.stopMotor();
        break;
      case CURRENT: // DO NOT USE, BROKEN
        masterVortexController.setSetpoint(
            FlywheelConstants.kDebuggingCurrent, ControlType.kCurrent, ClosedLoopSlot.kSlot0);
        break;
      case VELOCITY_SPARK:
        double RPMsetpoint = outputs.velocityRadsPerSec;
        // double arrfeedForward = ffcalculator.calculate(RPMsetpoint);
        masterVortexController.setSetpoint(
            RPMsetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        break;
      case VELOCITY_SETPOINT:
        double measuredVelocity = outputs.measuredVelocityRadPerSec;
        double setpoint = outputs.velocityRadsPerSec;

        double error = setpoint - measuredVelocity;
        Logger.recordOutput("Flywheel/Error", error);

        double acceleration = outputs.calculatedAcceleration;

        if (setpoint <= 0) {
          masterVortex.stopMotor();
          currentPhase = FlywheelPhase.IDLE; // Reset phase
          return;
        }

        if (Math.abs(error) <= FlywheelConstants.idleTolerance) {
          // Already at goal velocity
          currentPhase = FlywheelPhase.IDLE;
        } else if (acceleration < FlywheelConstants.ballDetectionThreshold) {
          // A drop in speed indicated a ball is in the shooter.
          // Note: Even if we were in the middle of recovering from a previous shot,
          // a negative spike will show a new ball is in the shooter.
          // Note: Velocity checks doesn't catch this.
          currentPhase = FlywheelPhase.BALL;
        } else if (currentPhase == FlywheelPhase.BALL && acceleration > 0) {
          // We were in the BALL phase but the velocity has stopped dropping
          // and is now positive. Ball has left the shooter.
          currentPhase = FlywheelPhase.RECOVERY;
        } else if (currentPhase != FlywheelPhase.BALL
            && Math.abs(error) > FlywheelConstants.idleTolerance) {
          // If we are far away from the setpoint, either spin up for the first time, we are or
          // actively
          // recovering.
          // Note: Recovery and Startup use same logic
          currentPhase = FlywheelPhase.STARTUP;
        }

        switch (currentPhase) {
          case STARTUP:
          case RECOVERY:
            // Bang-Bang Controller to maximize Duty-Cycle Output
            // If error > 0 (too slow), request +12V. If error < 0 (too fast), request -12V.
            // if (Math.abs(error) <= FlywheelConstants.idleTolerance * 2) {
            //   currentPhase = FlywheelPhase.IDLE;
            // } else {
            //   double outputVolts = (error > 0) ? 12.0 : 0;
            //   masterVortex.setVoltage(outputVolts);
            // }

            // Old
            double outputVolts = (error > 0) ? 12.0 : 0;
            masterVortex.setVoltage(outputVolts);
            break;
          case IDLE:
            if (error > 0) {
              // The wheel is at goal velocity. Apply current output as needed to overcome friction
              // only.
              // Keeps the wheel at goal velocity when we are already there.
              // double idleCurrent = (FlywheelConstants.kIdleVelocityLinearCoefficient * setpoint);
              // masterVortexController.setSetpoint(idleCurrent, ControlType.kCurrent);

              // Interpolator Key in Radians/Sec
              double idleVoltage = FlywheelConstants.IDLE_VOLTAGE_INTERPOLATOR.get(setpoint);
              masterVortex.setVoltage(idleVoltage);
            } else {
              // Over-peeked the goal velocity, allow it to passively reduce velocity
              masterVortex.setVoltage(0);
            }
            break;
          case BALL:
            // Constant Torque-CurrentFOC Simulated Fever Ah Solution at 2AM
            // ignore velocity entirely when ball is in the shooter, just shove the max amount of
            // current/torque to shooter as needed. Because it is a fixed current each time,
            // ball exit velocity theoretically should be the same even if the voltage sags.
            // double ballCurrent =
            //     (FlywheelConstants.kIdleVelocityLinearCoefficient * setpoint)
            //         + FlywheelConstants.kAntilag;
            // masterVortexController.setSetpoint(ballCurrent, ControlType.kVoltage);
            double idleVoltage =
                FlywheelConstants.IDLE_VOLTAGE_INTERPOLATOR.get(setpoint) + kAntilag;
            masterVortex.setVoltage(idleVoltage);
            break;
        }
        break;
      case VOLTAGE:
        masterVortex.setVoltage(outputs.voltage);
        break;
    }
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
    followerVortex.stopMotor();
  }

  @AutoLogOutput(key = "Flywheel/CurrentPhase")
  public FlywheelPhase getCurrentPhase() {
    return currentPhase;
  }
}
