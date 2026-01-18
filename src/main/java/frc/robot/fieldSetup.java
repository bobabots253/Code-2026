package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.TreeMap;

public class FieldSetup {
  // new Translation2d(0,0)
  public static Rotation3d oneEightyRotation = new Rotation3d(0, 0, 180);
  public static Rotation3d ninetyRotation = new Rotation3d(0, 0, 90);
  public static Rotation3d twoSeventyRotation = new Rotation3d(0, 0, 270);
  public static Rotation3d zero = new Rotation3d(0, 0, 0);

  public static final Translation3d blueHubCenter = new Translation3d(4.626, 4.035, 1.45); // meters
  public static final Translation3d redHubCenter = new Translation3d(11.915, 4.035, 1.45); // meters

  public static final TreeMap<Integer, Pose3d> Tags =
      new TreeMap<Integer, Pose3d>() {
        {
          put(
              1,
              new Pose3d(
                  Units.inchesToMeters(467.64),
                  Units.inchesToMeters(292.31),
                  Units.inchesToMeters(35.00),
                  oneEightyRotation));
          put(
              1,
              new Pose3d(
                  Units.inchesToMeters(467.64),
                  Units.inchesToMeters(292.31),
                  Units.inchesToMeters(35.00),
                  oneEightyRotation));
          put(
              2,
              new Pose3d(
                  Units.inchesToMeters(469.11),
                  Units.inchesToMeters(182.60),
                  Units.inchesToMeters(44.25),
                  ninetyRotation));
          put(
              3,
              new Pose3d(
                  Units.inchesToMeters(445.35),
                  Units.inchesToMeters(172.84),
                  Units.inchesToMeters(44.25),
                  oneEightyRotation));
          put(
              4,
              new Pose3d(
                  Units.inchesToMeters(445.35),
                  Units.inchesToMeters(158.84),
                  Units.inchesToMeters(44.25),
                  oneEightyRotation));
          put(
              5,
              new Pose3d(
                  Units.inchesToMeters(469.11),
                  Units.inchesToMeters(135.09),
                  Units.inchesToMeters(44.25),
                  twoSeventyRotation));
          put(
              6,
              new Pose3d(
                  Units.inchesToMeters(467.64),
                  Units.inchesToMeters(25.37),
                  Units.inchesToMeters(35.00),
                  oneEightyRotation));
          put(
              7,
              new Pose3d(
                  Units.inchesToMeters(470.59),
                  Units.inchesToMeters(25.37),
                  Units.inchesToMeters(35.00),
                  zero));
          put(
              8,
              new Pose3d(
                  Units.inchesToMeters(483.11),
                  Units.inchesToMeters(135.09),
                  Units.inchesToMeters(44.25),
                  twoSeventyRotation));
          put(
              9,
              new Pose3d(
                  Units.inchesToMeters(492.88),
                  Units.inchesToMeters(144.84),
                  Units.inchesToMeters(44.25),
                  zero));
          put(
              10,
              new Pose3d(
                  Units.inchesToMeters(492.88),
                  Units.inchesToMeters(158.84),
                  Units.inchesToMeters(44.25),
                  zero));
          put(
              11,
              new Pose3d(
                  Units.inchesToMeters(483.11),
                  Units.inchesToMeters(182.60),
                  Units.inchesToMeters(44.25),
                  ninetyRotation));
          put(
              12,
              new Pose3d(
                  Units.inchesToMeters(470.59),
                  Units.inchesToMeters(292.31),
                  Units.inchesToMeters(35.00),
                  zero));
          put(
              13,
              new Pose3d(
                  Units.inchesToMeters(650.92),
                  Units.inchesToMeters(251.47),
                  Units.inchesToMeters(21.75),
                  oneEightyRotation));
          put(
              14,
              new Pose3d(
                  Units.inchesToMeters(650.92),
                  Units.inchesToMeters(274.47),
                  Units.inchesToMeters(21.75),
                  oneEightyRotation));
          put(
              15,
              new Pose3d(
                  Units.inchesToMeters(650.90),
                  Units.inchesToMeters(170.22),
                  Units.inchesToMeters(21.75),
                  oneEightyRotation));
          put(
              16,
              new Pose3d(
                  Units.inchesToMeters(650.90),
                  Units.inchesToMeters(153.22),
                  Units.inchesToMeters(21.75),
                  oneEightyRotation));
          put(
              17,
              new Pose3d(
                  Units.inchesToMeters(183.59),
                  Units.inchesToMeters(25.37),
                  Units.inchesToMeters(35.00),
                  zero));
          put(
              18,
              new Pose3d(
                  Units.inchesToMeters(182.11),
                  Units.inchesToMeters(135.09),
                  Units.inchesToMeters(44.25),
                  twoSeventyRotation));
          put(
              19,
              new Pose3d(
                  Units.inchesToMeters(205.87),
                  Units.inchesToMeters(144.84),
                  Units.inchesToMeters(44.25),
                  zero));
          put(
              20,
              new Pose3d(
                  Units.inchesToMeters(205.87),
                  Units.inchesToMeters(158.84),
                  Units.inchesToMeters(44.25),
                  zero));
          put(
              21,
              new Pose3d(
                  Units.inchesToMeters(182.11),
                  Units.inchesToMeters(182.60),
                  Units.inchesToMeters(44.25),
                  ninetyRotation));
          put(
              22,
              new Pose3d(
                  Units.inchesToMeters(183.59),
                  Units.inchesToMeters(292.31),
                  Units.inchesToMeters(35.00),
                  zero));
          put(
              23,
              new Pose3d(
                  Units.inchesToMeters(180.64),
                  Units.inchesToMeters(292.31),
                  Units.inchesToMeters(35.00),
                  oneEightyRotation));
          put(
              24,
              new Pose3d(
                  Units.inchesToMeters(168.11),
                  Units.inchesToMeters(182.60),
                  Units.inchesToMeters(44.25),
                  ninetyRotation));
          put(
              25,
              new Pose3d(
                  Units.inchesToMeters(158.34),
                  Units.inchesToMeters(172.84),
                  Units.inchesToMeters(44.25),
                  oneEightyRotation));
          put(
              26,
              new Pose3d(
                  Units.inchesToMeters(158.34),
                  Units.inchesToMeters(158.84),
                  Units.inchesToMeters(44.25),
                  oneEightyRotation));
          put(
              27,
              new Pose3d(
                  Units.inchesToMeters(168.11),
                  Units.inchesToMeters(135.09),
                  Units.inchesToMeters(44.25),
                  twoSeventyRotation));
          put(
              28,
              new Pose3d(
                  Units.inchesToMeters(180.64),
                  Units.inchesToMeters(25.37),
                  Units.inchesToMeters(35.00),
                  oneEightyRotation));
          put(
              29,
              new Pose3d(
                  Units.inchesToMeters(0.30),
                  Units.inchesToMeters(26.22),
                  Units.inchesToMeters(21.75),
                  zero));
          put(
              30,
              new Pose3d(
                  Units.inchesToMeters(0.30),
                  Units.inchesToMeters(43.22),
                  Units.inchesToMeters(21.75),
                  zero));
          put(
              31,
              new Pose3d(
                  Units.inchesToMeters(0.32),
                  Units.inchesToMeters(147.47),
                  Units.inchesToMeters(21.75),
                  zero));
          put(
              32,
              new Pose3d(
                  Units.inchesToMeters(0.32),
                  Units.inchesToMeters(164.47),
                  Units.inchesToMeters(21.75),
                  zero));
        }
      };
}
