package frc.robot.subsystems.agitator;
import static frc.robot.subsystems.agitator.AgitatorConstants.*;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.agitator.AgitatorIO.AgitatorIOOutputMode;
import frc.robot.subsystems.agitator.AgitatorIO.AgitatorIOOutputs;
import frc.robot.util.FullSubsystem;
import lombok.RequiredArgsConstructor;

public class AgitatorSubsystem extends FullSubsystem{

    private final AgitatorIO io;
    private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();
    private final AgitatorIOOutputs outputs = new AgitatorIOOutputs();

    private Alert masterDisconnected;

@RequiredArgsConstructor
        public enum Goal {
            IDLE(() -> 0.0),
            DEPLOYED(()-> AgitatorConstants.intakingVolts),
            STOW(()-> AgitatorConstants.stowVolts),
            JUGGLE(()-> AgitatorConstants.jugglingVolts),
            DEBUGGING(()-> AgitatorConstants.debuggingVolts);

            private final DoubleSupplier voltage;



            private double getGoal(){
                return voltage.getAsDouble();

            }

        }

        @AutoLogOutput(key = "Agitator/Goal")
        private Goal currentGoal = Goal.IDLE;

        public AgitatorSubsystem(AgitatorIO io) {
            this.io = io;

            masterDisconnected = new Alert("Agitator motor disconnected", Alert.AlertType.kWarning);


            setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Agitator Idle"));


        }


        public void periodic() {
            io.updateInputs(inputs);
            Logger.processInputs("Agitator", inputs);

            masterDisconnected.set(Robot.showHardwareAlerts() && (!inputs.masterMotorConnected));

            if (DriverStation.isDisabled()) {
                setGoal(Goal.IDLE);


            }

            if(currentGoal == Goal.IDLE){
                stop();
            } else {
                runVoltage(currentGoal.getGoal());

            }
            }
@Override
public void periodicAfterScheduler() {
    Logger.recordOutput("Agitator/Mode", outputs.mode);
    io.applyOutputs(outputs);

}

private void setGoal(Goal desiredGoal){
    this.currentGoal = desiredGoal;
}



private void runVoltage(double voltage) {
outputs.mode = AgitatorIOOutputMode.VOLTAGE;
outputs.voltage = voltage;

}

@AutoLogOutput(key = "Agitator/MeasuredVoltage")
public double getVoltage() {
    return inputs.masterAppliedVolts;
}

public Command intakeCommand(){
return startEnd(() -> setGoal(Goal.DEPLOYED), () -> setGoal(Goal.IDLE))
.withName("Agitator Deploy");
}

public Command stowCommand() {
    return startEnd(() -> setGoal(Goal.STOW), () -> setGoal(Goal.IDLE)).withName("Agitator Stow");
}
 
public Command juggleCommand(){
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE)).withName("Agitator Juggle");

}

public Command debugCommand(){
    return startEnd(() -> setGoal(Goal.DEBUGGING), () -> setGoal(Goal.IDLE)).withName("Agitator Debug");

}

public Command stopCommand() {
    return runOnce( this::stop);
}

public Command runSetVoltageCommand(DoubleSupplier voltage ){
    return runEnd(() -> runVoltage(voltage.getAsDouble()), this::stop);
}

private void stop(){
    outputs.mode = AgitatorIOOutputMode.BRAKE;
}
}











