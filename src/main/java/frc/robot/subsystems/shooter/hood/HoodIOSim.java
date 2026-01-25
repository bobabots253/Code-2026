package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
  private static final DCMotor NeoMotorGroup = DCMotor.getNEO(2);

  // SingleJointedArmSim for Hood Simulation copied from Mechanical Advantage
  private final SingleJointedArmSim simulateSingleJointedArmSim =
      new SingleJointedArmSim(
          NeoMotorGroup, 1.0, .004, .33, 0.0, Units.degreesToRadians(85), false, 0);

  private static final double RadToDeg = 180.0 / Math.PI;
  private static final double DegToRad = Math.PI / 180.0;

  private double currentOutput = 0.0;
  private double appliedVolts = 0.0;
  private boolean currentControl = false;

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (currentControl) {
      appliedVolts =
          NeoMotorGroup.getVoltage(
              currentOutput, simulateSingleJointedArmSim.getVelocityRadPerSec());
    } else {
      appliedVolts = 0.0;
    }

    simulateSingleJointedArmSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    simulateSingleJointedArmSim.update(0.02);

    inputs.positionDegrees = simulateSingleJointedArmSim.getAngleRads() * RadToDeg;
    inputs.velocityDegPerSec = simulateSingleJointedArmSim.getVelocityRadPerSec() * RadToDeg;

    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = simulateSingleJointedArmSim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> {
        currentControl = false;
      }
      case COAST -> {
        currentOutput = 0.0;
        currentControl = true;
      }
      case CLOSED_LOOP -> {
        double targetPosRad = outputs.positionDegrees * DegToRad;

        // currentOutput =
        //     (simulateSingleJointedArmSim.getAngleRads() - targetPosRad) * outputs.kP
        //         + (simulateSingleJointedArmSim.getVelocityRadPerSec() -
        // outputs.targetVelRadPerSec) * outputs.kD;

        currentOutput =
            (targetPosRad - simulateSingleJointedArmSim.getAngleRads()) * outputs.kP
                + (-simulateSingleJointedArmSim.getVelocityRadPerSec()) * outputs.kD;

        currentControl = true;
      }
    }
  }
}
