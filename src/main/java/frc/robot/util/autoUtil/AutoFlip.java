package frc.robot.util.autoUtil;
import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.fieldSetup;

public class AutoFlip {

    
        public static double applyX(double x){
            return isRedAlliance() ? fieldSetup.fieldLength -x : x;
        }
        // '^' is the exclusive or operator.
        public static double applyY(double y, boolean isRightSide){
            return isRedAlliance() ^ isRightSide ? fieldSetup.fieldWidth -y : y;
        }
    
        public static Rotation2d applyRot(double rotation, boolean isRightSide) {
            if (!isRightSide) {
            return isRedAlliance() ? new Rotation2d((-rotation) + Math.PI) : new Rotation2d(rotation);
            } else return isRedAlliance() ? new Rotation2d(rotation + Math.PI) : new Rotation2d(-rotation);
            
  }

   public static boolean isRedAlliance(){
            return 
            DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }

}

