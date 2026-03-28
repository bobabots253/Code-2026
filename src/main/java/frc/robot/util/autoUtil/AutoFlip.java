package frc.robot.util.autoUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

public class AutoFlip {

    
        public static double applyX(double x){
            return isRedAlliance() ? FieldConstants.fieldLength -x : x;
        }
        // '^' is the exclusive or operator.
        public static double applyY(double y, boolean isRightSide){
            return isRedAlliance() ^ isRightSide ? FieldConstants.fieldWidth -y : y;
        }
    
        public static Rotation2d applyRot(Rotation2d rotation) {
            return isRedAlliance() ? rotation.rotateBy(new Rotation2d(0.0, Math.PI)) : rotation;
  }

   public static boolean isRedAlliance(){
            return 
            DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }

}

