package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.masterPositionConversionFactor;
import static frc.robot.subsystems.shooter.hood.HoodConstants.masterVelocityConversionFactor;
import static frc.robot.subsystems.shooter.hood.HoodConstants.sparkMasterFlyWheelkD;
import static frc.robot.subsystems.shooter.hood.HoodConstants.sparkMasterFlyWheelkI;
import static frc.robot.subsystems.shooter.hood.HoodConstants.sparkMasterFlyWheelkP;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;

public class HoodIOSpark implements HoodIO {
  // Declare REV motor hardware here
  private final SparkBase masterVortex;

  // Declare REV encoders hardware here
  private final AbsoluteEncoder masterAbsoluteEncoder;

  // Declare motor controllers
  private final SparkClosedLoopController masterVortexController;

  public HoodIOSpark() {
    // Initialize REV motor hardware here
    masterVortex =
        new SparkFlex(HoodConstants.sparkMasterHoodCanId, SparkBase.MotorType.kBrushless);

    // Initialize REV encoders hardware here
    masterAbsoluteEncoder = masterVortex.getAbsoluteEncoder();

    // Initialize motor controllers here
    masterVortexController = masterVortex.getClosedLoopController();

    var masterVortexConfig = new SparkFlexConfig();
    masterVortexConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(35); // Amps
    masterVortexConfig
        .encoder
        .positionConversionFactor(masterPositionConversionFactor)
        .velocityConversionFactor(masterVelocityConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    masterVortexConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(sparkMasterFlyWheelkP, sparkMasterFlyWheelkI, sparkMasterFlyWheelkD);
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
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Find a way to log master connection status
    inputs.positionRads = masterAbsoluteEncoder.getPosition();
    inputs.positionRadsPerSec = masterAbsoluteEncoder.getVelocity();
    inputs.appliedVoltage = masterVortex.getAppliedOutput();
    inputs.supplyCurrentAmps = masterVortex.getOutputCurrent();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST:
        masterVortex.stopMotor();
      case BRAKE:
        masterVortex.stopMotor();
      case CLOSED_LOOP:
        masterVortexController.setSetpoint(
            Units.radiansToRotations(outputs.positionRad),
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
