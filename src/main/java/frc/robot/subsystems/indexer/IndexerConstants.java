package frc.robot.subsystems.indexer;

public class IndexerConstants {
  public static final int sparkMasterIndexerCanId = 12;
  public static final int masterCurrentLimit = 50; // amps

  // Temporary Values (Confirm Functionality)
  public static final double stowVolts = 0.0;
  public static final double indexingVolts = -12.0;
  public static final double jugglingVolts = 0.0;
  public static final double debuggingVolts = 0.0;

  public static final double debuggingCurrent = 60;

  // Used as an early warning if the Indexer is jammed
  public static final double highCurrentThreshold = 50;
}
