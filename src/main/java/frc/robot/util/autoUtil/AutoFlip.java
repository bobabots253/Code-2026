package frc.robot.util.autoUtil;
import frc.robot.util.autoUtil.AutoCommandPicker;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

public class AutoFlip {
    public boolean autoFlipped(){
        return 
        DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
    public boolean sideFlipped(){
        return AutoCommandPicker.CommandName.isFlipped();
    }

    public double allianceFlipX(double x){
        return autoFlipped() ? FieldConstants.fieldLength -x : x;
    }

    public double allianceFlipY(double y){
        return autoFlipped() ? FieldConstants.fieldWidth -y : y;
    }

}

