package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class FlywheelIOSpark implements FlywheelIO {
  private final SparkBase flywheelMasterVortex;
  private final SparkBase flywheelFollowerVortex;
  private final BangBangController flywheelController = new BangBangController(flywheelTolerance); //creates new Bang Bang Controller with tolerance of 1. 
  private final Debouncer flywheelDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling); // creates debouncer so we aren't constantl checking values
  private final RelativeEncoder flywheelMasterEncoder;
  private final RelativeEncoder flywheelFollowerEncoder;
  private final SimpleMotorFeedforward ffCalculator = new SimpleMotorFeedforward(kS, kV, kA);// new feedforward, used to calculate ffvolts later on. Constants are placeholders.

  public FlywheelIOSpark() {
    flywheelMasterVortex = new SparkFlex(flywheelMasterCanID, MotorType.kBrushless); // creating 2 SparkFLexes to run the shooter. Note: Update CANIDs
    flywheelFollowerVortex = new SparkFlex(flywheelFollowerCanID, MotorType.kBrushless);
    flywheelMasterEncoder = flywheelMasterVortex.getEncoder(); //relative encoders to track velocity
    flywheelFollowerEncoder = flywheelFollowerVortex.getEncoder();
    var flywheelMasterConfig = new SparkFlexConfig(); //master config
    flywheelMasterConfig
        .idleMode(IdleMode.kCoast) //idle mode is coast
        .smartCurrentLimit(flywheelCurrentLimit) // sets current limit to vortex current limit, 50 amps
        .voltageCompensation(12.0);
    flywheelMasterConfig
        .encoder
        .inverted(flywheelEncoderInverted)
        .positionConversionFactor(flywheelEncoderPositionFactor); // converts encoder positions from rotations to radians
    flywheelMasterConfig
        .signals // sets signals to collect every 20ms to calculate inputs
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(//tries 5 times to configure the master motor with these parameters + the config
        flywheelMasterVortex,
        5,
        () ->
            flywheelMasterVortex.configure(
                flywheelMasterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    var flywheelFollowerConfig = new SparkFlexConfig();
    flywheelFollowerConfig //same config but for follower
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(flywheelCurrentLimit)
        .voltageCompensation(12.0)
        .follow(flywheelMasterVortex, true);
    flywheelFollowerConfig
        .encoder
        .inverted(true)// this encoder spins in the opposite directtion (I think), so we invert to compensate this
        .positionConversionFactor(flywheelEncoderPositionFactor);
    flywheelFollowerConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        flywheelFollowerVortex,
        5,
        () ->
            flywheelFollowerVortex.configure(
                flywheelFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sparkStickyFault = false; //clears spark faults
    ifOk(// proccesses a valid value from the sparkflex
        flywheelMasterVortex,
        new DoubleSupplier[] { // array of doubles w/o names to calculate volts
          flywheelMasterVortex::getAppliedOutput, flywheelMasterVortex::getBusVoltage
        },
        (values) -> inputs.flywheelMasterVolts = values[0] * values[1]); // calculates values for volts by multiplying duty cycle by bus voltage
    inputs.flywheelMasterCurrentAmps = flywheelMasterVortex.getOutputCurrent(); //gets amps input
    inputs.flywheelMasterPosRad = flywheelMasterEncoder.getPosition(); //gets position input
    inputs.flywheelMasterVelocityRad = flywheelMasterEncoder.getVelocity(); //gets velocity input
    inputs.flywheelMasterConnected = flywheelDebouncer.calculate(!sparkStickyFault); //checks for errors that persist for more than 0.5 sec to get connected input

    ifOk(// same as above but for follower.
        flywheelFollowerVortex,
        new DoubleSupplier[] {
          flywheelFollowerVortex::getAppliedOutput, flywheelFollowerVortex::getBusVoltage
        },
        (values) -> inputs.flywheelFollowerVolts = values[0] * values[1]);
    inputs.flywheelFollowerCurrentAmps = flywheelFollowerVortex.getOutputCurrent();
    inputs.flywheelFollowerPosRad = flywheelFollowerEncoder.getPosition();
    inputs.flywheelFollowerVelocityRad = flywheelFollowerEncoder.getVelocity();
    inputs.flywheelFollowerConnected = flywheelDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode) { //switch statement based on output mode
      case COAST: // if the motor is in coast mode, stop it
        flywheelMasterVortex.stopMotor();
        break;
      case VOLTAGE: // if the motor is in voltage mode, give it 6 volts. 6 is a placeholder value, 50% seemed safe.
        runVolts(6);
        break;
      case BANG_BANG: //if the motor is in BANG BANG mode, run the velocity bang bang controller and the feedforward, with a setpoint of velocityRAdPerSec from outputs
      runVelocityBangBang(outputs.velocityRadsPerSec);
        break;
    }
  }
@Override
public void runVolts(double volts) { // method to run voltage mode. Can be called indpendently of outputs, but also runs in outputs.
    flywheelMasterVortex.setVoltage(volts);
  }

public void runVelocityBangBang(double velocityRadsPerSec){ //method to run bang bang control with a feed forward. Can also be called independently of outputs.
    double ffVolts = ffCalculator.calculate(velocityRadsPerSec); // calculates feedforward based off of a velocity setpoint, omits acceleration because we have no stepoint here for bang bang (I think)
    flywheelMasterVortex.set(flywheelController.calculate(flywheelMasterEncoder.getVelocity(), velocityRadsPerSec + 0.9 * ffVolts));
// sets the flywheel speed using the ff and bang bang controller. FF is reduced slightly to compensate for the high power/ unpredictability of the bang bang controller, as recommended on WPILib docs.
// uses the relative encoder from earlier to measure this velocity
}



}
