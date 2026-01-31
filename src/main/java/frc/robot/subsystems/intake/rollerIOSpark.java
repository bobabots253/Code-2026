package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.rollerConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.intake.rollerIO.rollerIOInputs;

public class rollerIOSpark implements rollerIO {
    private final SparkBase rollerSpark;
    private final SparkClosedLoopController rollerController;
    private final Debouncer rollerDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final RelativeEncoder rollerEncoder;
    
    
    public rollerIOSpark(){
        rollerSpark = new SparkMax(rollerCanID, MotorType.kBrushless);
        rollerEncoder = rollerSpark.getEncoder();
        rollerController = rollerSpark.getClosedLoopController();
        var rollerConfig = new SparkMaxConfig();
        rollerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(rollerCurrentLimit)
        .voltageCompensation(12.0);
        rollerConfig
        .encoder
        .inverted(rollerEncoderInverted)
        .positionConversionFactor(rollerEncoderPositionFactor);
        rollerConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
        tryUntilOk(
            rollerSpark,
             5,
             () ->
            rollerSpark.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

  
}
@Override
  public void updateInputs(rollerIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        rollerSpark,
        new DoubleSupplier[] {rollerSpark::getAppliedOutput, rollerSpark::getBusVoltage},
        (values) -> inputs.rollerAppliedVolts = values[0] * values[1]);
    ifOk(rollerSpark, rollerSpark::getOutputCurrent, (value) -> inputs.rollerCurrentAmps = value);
    inputs.rollerConnected = rollerDebouncer.calculate(!sparkStickyFault);
}
@Override
public void setrollerOpenLoop(double input){
    rollerSpark.set(input);



    


}

}


    


