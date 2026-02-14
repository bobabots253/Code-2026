package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

public class HoodIOSpark implements HoodIO {
  private final SparkBase hoodSpark;
  private final RelativeEncoder hoodEncoder;
  private final SparkClosedLoopController hoodController;
  private final ProfiledPIDController profiledHoodController;
  private final Debouncer hoodDebounce;

  public HoodIOSpark() {
    hoodDebounce = new Debouncer(0.5, DebounceType.kFalling);
    hoodSpark = new SparkMax(sparkMasterHoodCanId, MotorType.kBrushless);
    hoodEncoder = hoodSpark.getEncoder();
    hoodController = hoodSpark.getClosedLoopController();

    profiledHoodController = new ProfiledPIDController(sparkHoodProfiledkP, sparkHoodProfiledkI, sparkHoodProfiledkD, 
      new TrapezoidProfile.Constraints(sparkHoodMaxAccel, sparkHoodMaxVelocity), 0.01);

    var hoodConfig = new SparkMaxConfig();
    hoodConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(hoodCurrentLimit)
        .voltageCompensation(12.0);
    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(minAngleRad, maxAngleRad)
        .pid(sparkHoodkP, sparkHoodkI, sparkHoodkD);
    hoodConfig
        .encoder
        .positionConversionFactor(kTotalReduction)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
    hoodConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        hoodSpark,
        5,
        () ->
            hoodSpark.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(HoodIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(
        hoodSpark,
        new DoubleSupplier[] {hoodSpark::getAppliedOutput, hoodSpark::getBusVoltage},
        (values) -> inputs.hoodVolts = values[0] * values[1]);
    ifOk(hoodSpark, hoodSpark::getOutputCurrent, (value) -> inputs.hoodCurrentAmps = value);
    inputs.hoodSparkConnected = hoodDebounce.calculate(!sparkStickyFault);
    ifOk(hoodSpark, hoodEncoder::getPosition, (value) -> inputs.hoodPosRad = value);
  }

  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE:
        hoodSpark.stopMotor();
        break;
      case VOLTAGE:
        setPercentVoltage(0.1);
      case CLOSED_LOOP_CONTROL:
        setPosition(Units.degreesToRotations(outputs.hoodSetPosRad));
      case ACCELERATION_CONTROL: 
        setPosition(Units.degreesToRotations(outputs.hoodSetPosRad));
    }
  }

  public void setPercentVoltage(double decimalPercent) {
    hoodSpark.set(decimalPercent);
  }

  // public Boolean isWithinRange(Boolean withinRange) {
  //   ? 
  // }
  public void setPosition(HoodIOOutputs outputs) {
      switch (outputs.mode) {
       case CLOSED_LOOP_CONTROL:
        hoodController.setSetpoint(outputs.hoodSetPosRad, ControlType.kPosition);
        break;
      case ACCELERATION_CONTROL:
        hoodSpark.setVoltage(profiledHoodController.calculate((hoodEncoder.getPosition() - hoodOffset), outputs.hoodSetPosRad));
        break;
    }
  }
  
  public Boolean atAngle(Boolean atAngle, double setpoint) {
    return atAngle;
  }
  }
//}