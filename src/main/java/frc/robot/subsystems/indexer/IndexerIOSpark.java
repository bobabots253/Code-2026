package frc.robot.subsystems.indexer;

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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class IndexerIOSpark implements IndexerIO {
  // Declare REV motor hardware here
  private final SparkBase masterNEO;

  private final SparkClosedLoopController masterNEOController;

  private final RelativeEncoder masterRelativeEncoder;

  // Declare WPILib Debouncer for Motor Disconnection Alerts here
  private final Debouncer masterNEODebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  public IndexerIOSpark() {
    // Initialize REV motor hardware here
    masterNEO = new SparkMax(IndexerConstants.sparkMasterIndexerCanId, MotorType.kBrushless);
    masterNEOController = masterNEO.getClosedLoopController();
    masterRelativeEncoder = masterNEO.getEncoder();
    SparkMaxConfig masterSparkMaxConfig = new SparkMaxConfig();
    masterSparkMaxConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IndexerConstants.masterCurrentLimit)
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    masterSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0, 0);
    tryUntilOk(
        masterNEO,
        5,
        () ->
            masterNEO.configure(
                masterSparkMaxConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Update all the IndexerIO inputs for the master motor
    // Use SparkStickyFaults to track motor connectivity (See SparkUtil for Implementation)
    sparkStickyFault = false;
    ifOk(
        masterNEO,
        new DoubleSupplier[] {masterNEO::getAppliedOutput, masterNEO::getBusVoltage},
        (values) -> inputs.masterAppliedVolts = values[0] * values[1]);
    ifOk(masterNEO, masterNEO::getOutputCurrent, (value) -> inputs.masterSupplyCurrentAmps = value);
    inputs.masterMotorConnected =
        masterNEODebouncer.calculate(!sparkStickyFault); // Force Connectivity Check
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {

    double setpoint = outputs.voltage;

    switch (outputs.mode) {
      case BRAKE:
        masterNEO.stopMotor();
        break;
      case VOLTAGE:
        masterNEO.setVoltage(setpoint);
        break;
      case CURRENT:
        masterNEOController.setSetpoint(outputs.current, ControlType.kCurrent);
    }
  }
}
