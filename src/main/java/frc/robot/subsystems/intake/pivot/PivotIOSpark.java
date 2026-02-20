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
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import java.util.function.DoubleSupplier;

public class PivotIOSpark implements PivotIO {
  // Declare REV motor hardware
  private final SparkBase masterVortex;

  // Declare relative encoder
  private final RelativeEncoder masterRelativeEncoder;

  // Declare closed loop controller
  private final SparkClosedLoopController masterVortexController;

  // Declare WPILib Debouncer for Motor Disconnection Alerts here
  private final Debouncer masterNEODebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  public PivotIOSpark() {
    masterVortex = new SparkFlex(sparkMasterPivotCanId, MotorType.kBrushless);
    masterRelativeEncoder = masterVortex.getEncoder();
    masterVortexController = masterVortex.getClosedLoopController();

    SparkMaxConfig masterVortexConfig = new SparkMaxConfig();
    masterVortexConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(pivotCurrentLimit);
    masterVortexConfig
        .encoder
        .positionConversionFactor(pivotPositionConversionFactor)
        .velocityConversionFactor(pivotVelocityConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    masterVortexConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(pivotkP, pivotkI, pivotkD);
    masterVortexConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
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

    // Assuming you never call reset position when you aren't supposed to
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    // Update all the FlywheelIO inputs for the master motor
    // Use SparkStickyFaults to track motor connectivity (See SparkUtil for Implementation)
    sparkStickyFault = false;
    ifOk(
        masterVortex,
        masterRelativeEncoder::getPosition,
        (value) -> inputs.masterPositionRads = value + HoodConstants.hoodOffsetRad);
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
        masterVortex::getAppliedOutput,
        (value) -> inputs.masterSupplyCurrentAmps = value);
    inputs.masterMotorConnected =
        masterNEODebouncer.calculate(!sparkStickyFault); // Force Connectivity Check
  }

  @Override
  public void applyOutputs(PivotIOOutputs outputs) {
    double setpoint = outputs.positionRad;

    switch (outputs.mode) {
      case BRAKE:
        masterVortex.stopMotor();
        break;
      case CLOSED_LOOP:
        masterVortexController.setSetpoint(
            setpoint, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        break;
    }
  }

  @Override
  public void stop() {
    masterVortex.set(0.0);
  }
}
