package frc.robot.subsystems.intake.roller;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;

public class RollerIOSpark implements RollerIO {
  // Declare REV motor hardware here
  private final SparkBase masterNEO;

  // Declare WPILib Debouncer for Motor Disconnection Alerts here
  private final Debouncer masterNEODebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  public RollerIOSpark() {
    // Initialize REV motor hardware here
    masterNEO = new SparkMax(RollerConstants.sparkMasterRollerCanId, MotorType.kBrushless);
    SparkMaxConfig masterSparkMaxConfig = new SparkMaxConfig();
    masterSparkMaxConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        masterNEO,
        5,
        () ->
            masterNEO.configure(
                masterSparkMaxConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters));
  }
}
