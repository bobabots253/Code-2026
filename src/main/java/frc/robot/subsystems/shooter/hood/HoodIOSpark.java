package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

public class HoodIOSpark implements HoodIO {
  private final SparkBase hoodSpark;
  private final RelativeEncoder hoodEncoder;
  private final SparkClosedLoopController hoodController;
  public HoodIOSpark () {
    hoodSpark = new SparkMax(sparkMasterHoodCanId, MotorType.kBrushless);
    hoodEncoder = hoodSpark.getEncoder();
    hoodController = hoodSpark.getClosedLoopController();

  }
}
