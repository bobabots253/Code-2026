package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor NeoMotorGroup = DCMotor.getNEO(2);
  private static final DCMotorSim simulatedNeoMotorGroup =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(NeoMotorGroup, 0.005, 1), NeoMotorGroup);

  private PIDController flywheelPidController = new PIDController(0.0002, 0, 0, 0.02);
  private double currentOutput = 0.0;
  private double currentOutputAsVolt = 0.0;
  private double appliedVolts = 0.0;

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    currentOutputAsVolt =
        NeoMotorGroup.getVoltage(
            currentOutput, simulatedNeoMotorGroup.getAngularVelocityRadPerSec());
    appliedVolts = currentOutputAsVolt;

    simulatedNeoMotorGroup.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    simulatedNeoMotorGroup.update(0.02);

    inputs.connected = true;
    inputs.velocityRadsPerSec = simulatedNeoMotorGroup.getAngularVelocityRadPerSec();
    inputs.flywheelRPM =
        simulatedNeoMotorGroup.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = simulatedNeoMotorGroup.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
  }

  @Override
  public void setFlywheelRPM(FlywheelIOOutputs outputs) {
    currentOutput =
        flywheelPidController.calculate(
            simulatedNeoMotorGroup.getAngularVelocityRadPerSec(), outputs.velocityRadsPerSec);
  }
}
