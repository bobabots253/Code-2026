package frc.robot.util.autoUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

public class AutoFlip {
    public static boolean autoFlipped(){
            return 
            DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }
    
        public double applyX(double x){
            return autoFlipped() ? FieldConstants.fieldLength -x : x;
        }
        // '^' is the exclusive or operator
        public double applyY(double y, boolean isRightSide){
            return autoFlipped() ^ isRightSide ? FieldConstants.fieldWidth -y : y;
        }
    
        public static Rotation2d applyRot(Rotation2d rotation) {
            return autoFlipped() ? rotation.rotateBy(new Rotation2d(0.0, Math.PI)) : rotation;
  }


}

