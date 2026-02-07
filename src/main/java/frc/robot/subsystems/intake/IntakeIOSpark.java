package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeIOSpark implements IntakeIO {
  // declaring rev hardware (SparkBase)
  private final SparkBase intakeSpark;
  // declaring closed loop controller
  private final SparkClosedLoopController intakeController;
  // private final Debouncer intakeDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  // declaring absolute encoder
  private final RelativeEncoder intakeEncoder;

  @SuppressWarnings("unused")
  private boolean WasCoasted = true;

  public IntakeIOSpark() {
    // intake canID differs other devices
    intakeSpark = new SparkFlex(intakeCanID, MotorType.kBrushless);
    // returns encoder
    intakeEncoder = intakeSpark.getEncoder();
    // Makes sparkFlex listen to closed loop controller when in closed loop mode
    intakeController = intakeSpark.getClosedLoopController();
    var intakeConfig = new SparkFlexConfig();
    intakeConfig
        // dot notations = settings
        // idleMode = setting that chooses what the motor does when not given a voltage
        .idleMode(IdleMode.kBrake)
        // makes sure motor doesn't overheat
        .smartCurrentLimit(intakeCurrentLimit);
    // forces voltage to balance out
    // .voltageCompensation(12.0);
    intakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // records past 1 rotation
        .positionWrappingEnabled(true)
        // max recorded rotations past 1
        // .positionWrappingInputRange(intakePIDMinOutput, intakePIDMaxOutput)
        // get to target quickly, slow down when close
        .pid(intakekP, intakekI, intakekD);
    intakeConfig
        .encoder
        .inverted(intakeEncoderInverted)
        .positionConversionFactor(intakeEncoderPositionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    intakeConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) 5)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        intakeSpark,
        5,
        () ->
            intakeSpark.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    tryUntilOk(intakeSpark, 5, () -> intakeEncoder.setPosition(0));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = intakeEncoder.getPosition();
    inputs.velocityRad = intakeEncoder.getVelocity();
    inputs.appliedVolts = intakeSpark.getAppliedOutput() * intakeSpark.getBusVoltage();
    inputs.currentAmps = intakeSpark.getOutputCurrent();
  }

  @Override
  public void updateOutputs(IntakeIOOutputs outputs) {
    if (outputs.mode == intakeIOOutputMode.COAST) {
      intakeSpark.stopMotor();
    } else {
      setSpeed(0.1);
    }
  }

  public void setSpeed(double speed) {
    intakeSpark.set(speed);
  }

  public void setIntakePosition(double setpoint) {
    intakeController.setSetpoint(setpoint, SparkBase.ControlType.kPosition);
  }
}
