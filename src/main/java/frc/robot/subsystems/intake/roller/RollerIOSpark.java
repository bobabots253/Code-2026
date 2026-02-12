package frc.robot.subsystems.intake.roller;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.intake.roller.RollerIO.RollerIOInputs;

public class RollerIOSpark implements RollerIO {
  private final SparkBase rollerBase;

  public RollerIOSpark() {
    rollerBase = new SparkMax(11, MotorType.kBrushless);
    SparkMaxConfig rollerSparkConfig = new SparkMaxConfig();
    rollerSparkConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        rollerBase,
        5,
        () ->
            rollerBase.configure(
                rollerSparkConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters));
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerCurrentAmps = rollerBase.getOutputCurrent();
    inputs.rollerAppliedVolts = rollerBase.getAppliedOutput() * rollerBase.getBusVoltage();
  }

  @Override
  public void setRollerOpenLoop(double speed) {
    rollerBase.set(speed);
  }
}
