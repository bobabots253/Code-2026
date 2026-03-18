package frc.robot.util.swerveUtil;

/**
 * A less momemory intensive, primitive-based queue intended to replace Queue<Double>. in
 * SparkOdometryThread. Java collections cannot hold primitive types and will auto-wrap into a
 * Double object. Java streams also generate a lot of backend handlers. Memory is pre-allocated.
 */
public class PrimitiveDoubleQueue {
  private final double[] buffer;
  private int count;

  public PrimitiveDoubleQueue(int capacity) {
    this.buffer = new double[capacity];
    this.count = 0;
  }

  public void offer(double value) {
    if (count < buffer.length) {
      buffer[count++] = value;
    }
  }

  public double[] toArray() {
    double[] result = new double[count];
    System.arraycopy(buffer, 0, result, 0, count);
    return result;
  }

  public void clear() {
    count = 0;
  }
}
