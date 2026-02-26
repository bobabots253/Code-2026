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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class FlywheelIOSpark implements FlywheelIO {
  private final SparkBase masterVortex; // initializing the SparkFlexes for the shooter flywheel
  private final SparkBase followerVortex;
  private final SparkClosedLoopController
      flywheelController; // creates new Bang Bang Controller with tolerance of 1.

  private final Debouncer flywheelDebouncer =
      new Debouncer(
          0.5,
          Debouncer.DebounceType
              .kFalling); // creates debouncer so we aren't constantl checking values
  private final RelativeEncoder
      flywheelMasterRelativeEncoder; // initializing the relative encoders for the shooter flywheel
  private final RelativeEncoder flywheelFollowerRelativeEncoder;
  private final SimpleMotorFeedforward ffCalculator =
      new SimpleMotorFeedforward(
          kS, kV,
          kA); // new feedforward, used to calculate ffvolts later on. Constants are placeholders.

  public enum FlywheelState {
    STARTUP,
    IDLE,
    BALL,
    RECOVERY;
  }

  private FlywheelState currentState = FlywheelState.STARTUP;
  private double lastMeasuredVelocity = 0.0;
  private double lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

  public FlywheelIOSpark() {
    masterVortex =
        new SparkFlex(
            flywheelMasterCanID,
            MotorType.kBrushless); // creating 2 SparkFLexes to run the shooter.
    followerVortex = new SparkFlex(flywheelFollowerCanID, MotorType.kBrushless);
    flywheelMasterRelativeEncoder =
        masterVortex
            .getEncoder(); // relative encoders to track velocity. Does not get the velocity, just
    // specifies that the encoder tracks it.
    flywheelFollowerRelativeEncoder = followerVortex.getEncoder();
    flywheelController = masterVortex.getClosedLoopController();

    var flywheelMasterConfig = new SparkFlexConfig(); // master config
    flywheelMasterConfig
        .idleMode(IdleMode.kCoast) // idle mode is coast
        .smartCurrentLimit(
            flywheelCurrentLimit) // sets current limit to vortex current limit, 50 amps
        .voltageCompensation(
            12.0); // compensates for the drop in voltage when we draw more current to power the
    // motor.

    flywheelMasterConfig
        .encoder
        .inverted(flywheelEncoderInverted)
        .positionConversionFactor(flywheelEncoderPositionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2); // converts encoder velocities from rotations to radians

    flywheelMasterConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(flywheelVelocitykP, flywheelVelocitykI, flywheelVelocitykD);

    flywheelMasterConfig
        .signals // sets signals to collect every 20ms to calculate inputs and encoder values
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(5) // calls every 5 ms
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk( // tries 5 times to configure the master motor with these parameters + the config
        masterVortex,
        5,
        () ->
            masterVortex.configure(
                flywheelMasterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    var flywheelFollowerConfig = new SparkFlexConfig();
    flywheelFollowerConfig // same config but for follower
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(flywheelCurrentLimit)
        .voltageCompensation(12.0)
        .follow(masterVortex, true);

    flywheelFollowerConfig
        .encoder
        .inverted(
            true) // this encoder spins in the opposite direction, so we invert to compensate this
        .velocityConversionFactor(flywheelEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    flywheelFollowerConfig
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
                flywheelFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sparkStickyFault = false; // clears spark faults
    ifOk( // proccesses a valid value from the sparkflex
        masterVortex,
        new DoubleSupplier[] { // array of doubles w/o names to calculate volts
          masterVortex::getAppliedOutput, masterVortex::getBusVoltage
        },
        (values) ->
            inputs.flywheelMasterVolts =
                values[0]
                    * values[
                        1]); // calculates values for volts by multiplying duty cycle by bus voltage
    inputs.flywheelMasterCurrentAmps = masterVortex.getOutputCurrent(); // gets amps input
    inputs.flywheelMasterVelocityRad =
        flywheelMasterRelativeEncoder.getVelocity(); // gets velocity input
    inputs.flywheelMasterConnected =
        flywheelDebouncer.calculate(
            !sparkStickyFault); // checks for errors that persist for more than 0.5 sec to get
    // connected input

    ifOk( // same as above but for follower.
        followerVortex,
        new DoubleSupplier[] {followerVortex::getAppliedOutput, followerVortex::getBusVoltage},
        (values) -> inputs.flywheelFollowerVolts = values[0] * values[1]);
    inputs.flywheelFollowerCurrentAmps = followerVortex.getOutputCurrent();
    inputs.flywheelFollowerVelocityRad = flywheelFollowerRelativeEncoder.getVelocity();
    inputs.flywheelFollowerConnected = flywheelDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode) { // switch statement based on output mode
      case COAST: // if the motor is in coast mode, stop it
        masterVortex.stopMotor();
        break;
      case VOLTAGE: // if the motor is in voltage mode, give it volts = to setpoint
        runVolts(outputs.volts);
        break;
      case BANG_BANG: // if the motor is in BANG BANG mode, run the velocity setpoint and the
        // feedforward, with a setpoint of velocityRadPerSec from outputs
        double measuredVelocity = outputs.measuredVelocityRadPerSec;
        double setpoint = outputs.velocityRadsPerSec;
        double error = setpoint - measuredVelocity;

        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double deltaTime = currentTime - lastTime;
        double acceleration = (measuredVelocity - lastMeasuredVelocity) / deltaTime;
        lastMeasuredVelocity = measuredVelocity;
        lastTime = currentTime;

        if (setpoint <= 0) {
          masterVortex.stopMotor();
          currentState = FlywheelState.IDLE;
          return;
        }

        if (Math.abs(error) >= flywheelTolerance) {
          currentState = FlywheelState.IDLE;

        } else if (acceleration < ballDetectionThreshold) {
          currentState = FlywheelState.BALL;

        } else if (currentState == FlywheelState.BALL && acceleration > 0) {
          currentState = FlywheelState.RECOVERY;

        } else if (currentState != FlywheelState.BALL && Math.abs(error) > idleTolerance) {
          currentState = FlywheelState.STARTUP;
        }

        switch (currentState) {
          case STARTUP:
          case RECOVERY:
            if (error > 0) {
              double outputVolts = 12.0;
              masterVortex.set(outputVolts);

            } else if (error < 0) {
              double outputVolts = -12.0;
              masterVortex.set(outputVolts);
            }
            break;

          case IDLE:
            if (error > 0) {
              double idleCurrent = (kI_velocity * setpoint);
              flywheelController.setSetpoint(idleCurrent, ControlType.kCurrent);
            } else {
              masterVortex.setVoltage(0);
            }
            break;

          case BALL:
            double ballCurrent = (kI_velocity * setpoint) + kAntilag;
            flywheelController.setSetpoint(ballCurrent, ControlType.kCurrent);
            break;
        }
        break;
    }
  }

  @Override
  public void runVolts(
      double volts) { // method to run voltage mode. Can be called indpendently of outputs, but also
    // runs in outputs.
    masterVortex.setVoltage(volts);
  }

  @Override
  public void runVelocitySetpoint(
      double velocityRadsPerSec) { // method to run with a velocity stepoint. Can also be
    // called independently of outputs.
    double ffVolts = ffCalculator.calculate(velocityRadsPerSec);

    flywheelController.setSetpoint(velocityRadsPerSec + ffVolts, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    masterVortex.stopMotor();
    followerVortex.stopMotor();
  }
}
