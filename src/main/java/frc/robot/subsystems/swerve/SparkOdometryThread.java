package frc.robot.subsystems.swerve;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.swerveUtil.PrimitiveDoubleQueue;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class SparkOdometryThread {
  private final List<SparkBase> sparks = new ArrayList<>();
  private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();

  // Using our primitive queue to avoid GC lag
  private final List<PrimitiveDoubleQueue> sparkQueues = new ArrayList<>();
  private final List<PrimitiveDoubleQueue> genericQueues = new ArrayList<>();
  private final List<PrimitiveDoubleQueue> timestampQueues = new ArrayList<>();

  private static SparkOdometryThread instance = null;
  private Notifier notifier = new Notifier(this::run);
  private double[] sparkValuesCache = new double[0];
  private double[] genericValuesCache = new double[0];

  public static SparkOdometryThread getInstance() {
    if (instance == null) instance = new SparkOdometryThread();
    return instance;
  }

  private SparkOdometryThread() {
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (timestampQueues.size() > 0) {
      sparkValuesCache = new double[sparkSignals.size()];
      genericValuesCache = new double[genericSignals.size()];
      notifier.startPeriodic(1.0 / SwerveConstants.odometryFrequency);
    }
  }

  public PrimitiveDoubleQueue registerSignal(SparkBase spark, DoubleSupplier signal) {
    PrimitiveDoubleQueue queue = new PrimitiveDoubleQueue(20);
    SwerveSubsystem.odometryLock.lock();
    try {
      sparks.add(spark);
      sparkSignals.add(signal);
      sparkQueues.add(queue);
    } finally {
      SwerveSubsystem.odometryLock.unlock();
    }
    return queue;
  }

  public PrimitiveDoubleQueue registerSignal(DoubleSupplier signal) {
    PrimitiveDoubleQueue queue = new PrimitiveDoubleQueue(20);
    SwerveSubsystem.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      SwerveSubsystem.odometryLock.unlock();
    }
    return queue;
  }

  public PrimitiveDoubleQueue makeTimestampQueue() {
    PrimitiveDoubleQueue queue = new PrimitiveDoubleQueue(20);
    SwerveSubsystem.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      SwerveSubsystem.odometryLock.unlock();
    }
    return queue;
  }

  private void run() {
    // Read hardware outside the lock to prevent stalling the main loop
    double timestamp = RobotController.getFPGATime() / 1e6;

    for (int i = 0; i < genericSignals.size(); i++) {
      genericValuesCache[i] = genericSignals.get(i).getAsDouble();
    }

    // Local variable to track spark error
    boolean isValid = true;

    for (int i = 0; i < sparkSignals.size(); i++) {
      sparkValuesCache[i] = sparkSignals.get(i).getAsDouble();
      if (sparks.get(i).getLastError() != REVLibError.kOk) {
        isValid = false;
      }
    }

    if (!isValid) return;

    // Lock quickly just to push values to the queues
    SwerveSubsystem.odometryLock.lock();
    try {
      for (int i = 0; i < sparkSignals.size(); i++) {
        sparkQueues.get(i).offer(sparkValuesCache[i]);
      }
      for (int i = 0; i < genericSignals.size(); i++) {
        genericQueues.get(i).offer(genericValuesCache[i]);
      }
      for (int i = 0; i < timestampQueues.size(); i++) {
        timestampQueues.get(i).offer(timestamp);
      }
    } finally {
      SwerveSubsystem.odometryLock.unlock();
    }
  }
}
