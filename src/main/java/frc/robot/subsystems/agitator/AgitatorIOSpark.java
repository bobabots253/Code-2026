package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;

import com.revrobotics.spark.config.SparkMaxConfig;

// instantiates motor and debouncer
public class AgitatorIOSpark implements AgitatorIO{

private final SparkBase masterNEO;

private final Debouncer masterNEODebouncer = new Debouncer(0.25,Debouncer.DebounceType.kFalling);

public AgitatorIOSpark(){

// Configures agitator with listed settings
masterNEO = new SparkMax(agitatorCanID, MotorType.kBrushless);

SparkMaxConfig masterSparkMaxConfig = new SparkMaxConfig();
masterSparkMaxConfig.idleMode(IdleMode.kBrake)
.smartCurrentLimit(50)
.signals
.appliedOutputPeriodMs(20)
.busVoltagePeriodMs(20)
.outputCurrentPeriodMs(20);
tryUntilOk(masterNEO, 5, ()-> masterNEO.configure(
    masterSparkMaxConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kNoPersistParameters));
}

@Override
//sets inputs for aggitator
public void updateInputs(AgitatorInputs inputs){
    sparkStickyFault = false;
    ifOk(masterNEO,
    new DoubleSupplier[] {masterNEO::getAppliedOutput,masterNEO::getBusVoltage},(values) -> inputs.masterAppliedVolts = values[0] * values[1]);
    ifOk(masterNEO,masterNEO::getAppliedOutput, (value) ->inputs.masterSupplyCurrentAmps = value);
    inputs.masterMotorConnected = masterNEODebouncer.calculate(!sparkStickyFault);
inputs.masterAppliedVolts = masterNEO.getAppliedOutput();
inputs.masterSupplyCurrentAmps = masterNEO.getOutputCurrent();
}

@Override
// Sets voltage for agitator motor
public void applyOutputs(AgitatorIOOutputs outputs) {

    double setpoints = outputs.voltage;

    switch(outputs.mode){
        case BRAKE:
        masterNEO.stopMotor();
        break;
        case VOLTAGE:
        masterNEO.setVoltage(setpoints);
        break;
    }
}
}
