package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

public class IntakeIOSpark implements IntakeIO {
  private final SparkBase intakeSpark;
  private final SparkBaseConfig intakeConfig;

  private final Debouncer intakeConnectedDebounce = new Debouncer(intakeDebounceTime, Debouncer.DebounceType.kFalling);

  public IntakeIOSpark() {
    intakeSpark = new SparkMax(intakeCanId, MotorType.kBrushless);

    intakeConfig = new SparkMaxConfig();
    intakeConfig
      .idleMode(IdleMode.kBrake)
      .inverted(intakeInverted)
      .smartCurrentLimit(intakeCurrentLimit);

    tryUntilOk(
      intakeSpark,
      5, 
      () -> intakeSpark.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    );
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(
      intakeSpark, 
      new DoubleSupplier[] {intakeSpark::getAppliedOutput, intakeSpark::getBusVoltage}, 
      (values) -> inputs.intakeAppliedVolts = values[0] * values[1]
    );
    
    ifOk(
      intakeSpark,
      intakeSpark::getOutputCurrent, 
      (value) -> inputs.intakeCurrentAmps = value
    );

    inputs.intakeConnected = intakeConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setIntakeOpenLoop(double output) {
    intakeSpark.setVoltage(output);
  }
}
