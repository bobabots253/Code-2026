package frc.robot.subsystems.intake.roller;

import static frc.robot.util.SparkUtil.tryUntilOk;
import static frc.robot.subsystems.intake.roller.RollerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.intake.roller.RollerIO.RollerIOInputs;

public class RollerIOSpark implements RollerIO {
  private final DigitalInput rollerBeamBreak;
  private final SparkBase rollerSpark;

  public RollerIOSpark() {
    rollerSpark = new SparkMax(11, MotorType.kBrushless);
    rollerBeamBreak = new DigitalInput(beamBreakChannel); 


    SparkMaxConfig rollerSparkConfig = new SparkMaxConfig();
    rollerSparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        rollerSpark,
        5,
        () ->
            rollerSpark.configure(
                rollerSparkConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters));
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerCurrentAmps = rollerSpark.getOutputCurrent();
    inputs.rollerAppliedVolts = rollerSpark.getAppliedOutput() * rollerSpark.getBusVoltage();
    inputs.RollerBeamBreak = rollerBeamBreak.get();
  }

  @Override
  public void setRollerOpenLoop(double speed) {
    rollerSpark.set(speed);
  }

  public void updateOutputs(RollerIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST:
      rollerSpark.stopMotor();
      break;
      case VOLTAGE:
      rollerSpark.set(outputs.rollerSpeed);
      break;
    }
  }

  public boolean hasFuel() {
    return rollerBeamBreak.get();
  }
}


