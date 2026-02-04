package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class IntakeIOSpark implements IntakeIO {
  private final SparkBase intakeSpark;
  private final SparkClosedLoopController intakeController;
  private final Debouncer intakeDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final AbsoluteEncoder intakeEncoder;

  public IntakeIOSpark() {
    intakeSpark = new SparkMax(intakeCanID, MotorType.kBrushless);
    intakeEncoder = intakeSpark.getAbsoluteEncoder();
    intakeController = intakeSpark.getClosedLoopController();
    var intakeConfig = new SparkMaxConfig();
    intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(intakeCurrentLimit)
        .voltageCompensation(12.0);
    intakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(intakePIDMinOutput, intakePIDMaxOutput)
        .pid(intakekP, intakekI, intakekD);
    intakeConfig
        .encoder
        .inverted(intakeEncoderInverted)
        .positionConversionFactor(intakeEncoderPositionFactor);
    intakeConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        intakeSpark,
        5,
        () ->
            intakeSpark.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }


        
    
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        intakeSpark,
        new DoubleSupplier[] {intakeSpark::getAppliedOutput, intakeSpark::getBusVoltage},
        (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
    ifOk(intakeSpark, intakeSpark::getOutputCurrent, (value) -> inputs.intakeCurrentAmps = value);
    inputs.intakeConnected = intakeDebouncer.calculate(!sparkStickyFault);
    ifOk(intakeSpark, intakeEncoder::getPosition, (value) -> inputs.intakePosition = value);
  }

  
  public void setIntakePosition(double setpoint) {
    intakeController.setSetpoint(setpoint, SparkBase.ControlType.kPosition);
  }

}