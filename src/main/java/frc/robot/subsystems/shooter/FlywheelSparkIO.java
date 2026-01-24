package frc.robot.subsystems.shooter;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.subsystems.shooter.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Encoder;

public class FlywheelSparkIO implements FlywheelIO{
    private final SparkBase flywheelSpark;
    private final SparkClosedLoopController flywheelController;
    private final Debouncer flywheelDebouncer = 
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final RelativeEncoder flywheelEncoder;
    //private final SimpleMotorFeedforward flywheelFF;

    public FlywheelSparkIO(){ 
        flywheelSpark = new SparkMax(flywheelCanID, MotorType.kBrushless);
        flywheelEncoder = flywheelSpark.getEncoder();
        flywheelController = flywheelSpark.getClosedLoopController();
        var flywheelConfig = new SparkMaxConfig();
            flywheelConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(flywheelCurrentLimit)
            .voltageCompensation(12.0);
        flywheelConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(flywheelPIDMinOutput, flywheelPIDMaxOutput)
            .pid(flywheelVelocitykP,flywheelVelocitykI , flywheelVelocitykD);
        flywheelConfig
            .encoder
            .inverted(flywheelEncoderInverted)
            .positionConversionFactor(flywheelEncoderPositionFactor);
        flywheelConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        flywheelSpark,
        5,
        () ->
            flywheelSpark.configure(
                flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    
}
    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        flywheelSpark,
        new DoubleSupplier[] {flywheelSpark::getAppliedOutput, flywheelSpark::getBusVoltage},
        (values) -> inputs.flywheelAppliedVolts = values[0] * values[1]);
    ifOk(flywheelSpark, flywheelSpark::getOutputCurrent, (value) -> inputs.flywheelCurrentAmps = value);
    inputs.flywheelConnected = flywheelDebouncer.calculate(!sparkStickyFault);
    ifOk(flywheelSpark, flywheelEncoder::getVelocity, (value) -> inputs.flywheelVelocity = value);
    inputs.flywheelConnected = flywheelDebouncer.calculate(!sparkStickyFault);

    }
     public void setRPM(double velocity) {    
         double setPoint =
        MathUtil.inputModulus(flywheelEncoder.getPosition(), flywheelPIDMinOutput, flywheelPIDMaxOutput);
        flywheelController.setSetpoint(setPoint, ControlType.kVelocity);
    
}
}