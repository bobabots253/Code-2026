package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CrossBumpCommand extends Command {

  // ----------------------------------Direction Enum------------------------------//
  public enum CrossDirection {
    // from alliance zone toward the neutral zone.
    TO_NEUTRAL,

    // from neutral zone toward the alliance zone.
    TO_ALLIANCE
  }

  // ----------------------------------Internal State Machine------------------------------//

  private enum BumpState {

    // Driving at approach speed before any pitch is detected.
    // Exits when pitch exceeds PITCH_ENTRY_THRESHOLD_RAD.
    APPROACHING,

    // Exits when pitch velocity crosses zero (peak detected)
    CLIMBING,

    //  Exits when pitch drops below -PITCH_ENTRY_THRESHOLD_RAD (descent confirmed)
    //  or if peak pitch was small (robot straddled >:| the upward ramp).
    CRESTING,

    //  Continue at BUMP_SPEED_MS.
    //  Exits when pitch rises above -PITCH_FLAT_THRESHOLD_RAD for SETTLE_DEBOUNCE_SECONDS (all
    //  wheels back on flat ground).
    DESCENDING,

    // Command ends after SETTLE_DEBOUNCE_SECONDS of flat pitch.
    LEVELING,

    // isFinished() returns true.
    COMPLETE,

    /**
     * Safety state: bump was not detected within APPROACH_TIMEOUT_SECONDS. Robot may have stopped,
     * missed the bump, or negligently our pitch sign is inverted. isFinished() returns true to
     * prevent blocking the Command scheduler since we have a addRequirements call.
     */
    TIMED_OUT
  }

  // ----------------------------------Internal Constants ------------------------------//

  public static final class BumpConstants {

    // Start at 1.0 m/s/low value and tune up if the robot stalls on the ramp edge.
    public static double APPROACH_SPEED_MS = 4.0;

    public static double BUMP_SPEED_MS = 5.0;

    // Speed during the LEVELING phase (all wheels back on flat ground) in m/s.
    public static double SETTLE_SPEED_MS = 0.4;

    /**
     * Pitch threshold in radians to detect ramp entry (APPROACHING to CLIMBING). 3 degrees = 0.052
     * rad. Too low: false positives. Too high: robot is already significantly on the ramp before
     * detecting it.
     */
    public static double PITCH_ENTRY_THRESHOLD_RAD = Units.degreesToRadians(3.0);

    /**
     * Minimum peak pitch to confirm a real bump. 5 degrees = 0.087 rad. Prevents false cresting
     * detection on flat ground. The bump's actual peak is 8.35 deg (AI found) This threshold should
     * be below 8.35 deg to catch cases if the robot crosses near the ramp edge.
     */
    public static double PITCH_PEAK_MIN_RAD = Units.degreesToRadians(5.0);

    /** Use something small like 1.5 degrees */
    public static double PITCH_FLAT_THRESHOLD_RAD = Units.degreesToRadians(1.5);

    // How long in [seconds] our pitch must remain below PITCH_FLAT_THRESHOLD_RAD before the command
    // ends.
    public static double SETTLE_DEBOUNCE_SECONDS = 0.15;

    public static double APPROACH_TIMEOUT_SECONDS = 4.0;

    // 6.0 seconds is already very generous
    public static double TOTAL_TIMEOUT_SECONDS = 8.0;

    // When pitch velocity drops below this value (near zero or negative),
    // the robot is at or past the peak.
    public static double PITCH_PEAK_VELOCITY_THRESHOLD_RAD_PER_SEC = Units.degreesToRadians(0.5);
  }

  // ----------------------------------Begin Here------------------------------//

  private final SwerveSubsystem drive;
  private final CrossDirection direction;

  // +1 for TO_NEUTRAL, -1 for TO_ALLIANCE.
  private double driveSign;
  private double pitchSign;

  private BumpState state = BumpState.APPROACHING;

  // Peak pitch magnitude observed during the CLIMBING phase (radians).
  private double peakPitchRad = 0.0;

  // Timer started when the command begins, used for both timeouts.
  private final Timer totalTimer = new Timer();

  /** Timer started when APPROACHING begins, used for approach timeout. */
  private final Timer approachTimer = new Timer();

  // I'm sure there are better ways to do this but I have familiairty with debouncers
  private final Debouncer flatDebouncer =
      new Debouncer(BumpConstants.SETTLE_DEBOUNCE_SECONDS, DebounceType.kRising);

  // Signing of this command is decided in construction time (not part of state machine)
  public CrossBumpCommand(SwerveSubsystem drive, CrossDirection direction) {
    this.drive = drive;
    this.direction = direction;
    // Don't add in the drive sign early into construction, FMS not ready
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    state = BumpState.APPROACHING;
    peakPitchRad = 0.0;

    totalTimer.reset();
    totalTimer.start();
    approachTimer.reset();
    approachTimer.start();

    // Reset debouncer to ensure it starts in the not-debounced state
    flatDebouncer.calculate(false);

    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

    double allianceMultiplier = isRed ? -1.0 : 1.0;
    double directionMultiplier = (direction == CrossDirection.TO_NEUTRAL) ? 1.0 : -1.0;

    // Pre-Calculate driveSign to prevent double signing logic in run-time
    this.driveSign = directionMultiplier * allianceMultiplier;

    Logger.recordOutput("CrossBump/CalculatedDriveSign", driveSign);
    Logger.recordOutput("CrossBump/CalculatedPitchSign", pitchSign);
    Logger.recordOutput("CrossBump/Direction", direction.name());
    Logger.recordOutput("CrossBump/State", state.name());
  }

  // The use of transitionTo() makes the logic much easier to read and ensures consistent logging on
  // state changes.
  // NOTE: Use this is as a gold standard template
  @Override
  public void execute() {
    // Raw data from SwerveSubsystem
    double rawPitch = drive.getPitchPositionRad();
    double rawPitchRate = drive.getPitchVelocityRadPerSec();

    double pitch = rawPitch * driveSign;
    double pitchRate = rawPitchRate * driveSign;
    double absPitch = Math.abs(pitch);

    // total timeout for safety
    if (totalTimer.hasElapsed(BumpConstants.TOTAL_TIMEOUT_SECONDS)) {
      transitionTo(BumpState.TIMED_OUT);
      drive.stop();
      logState(pitch, pitchRate, 0.0);
      return;
    }

    double speedMs;

    switch (state) {
      case APPROACHING:
        speedMs = BumpConstants.APPROACH_SPEED_MS;

        // Approach timeout for bump not reached
        if (approachTimer.hasElapsed(BumpConstants.APPROACH_TIMEOUT_SECONDS)) {
          transitionTo(BumpState.TIMED_OUT);
          drive.stop();
          break;
        }

        // Bump detected: pitch is rising and exceeds entry threshold.
        // Important: Also check that the pitch sign matches the direction of travel:
        // Might be better: just check |pitch| > threshold regardless of sign
        if (pitch > BumpConstants.PITCH_ENTRY_THRESHOLD_RAD) {
          peakPitchRad = absPitch; // peak tracker
          transitionTo(BumpState.CLIMBING);
        }
        break;

      case CLIMBING:
        speedMs = BumpConstants.BUMP_SPEED_MS;

        // Track the peak pitch so we can confirm our crossing was a real bump
        if (absPitch > peakPitchRad) {
          peakPitchRad = absPitch;
        }

        // Apex detection: pitch velocity has crossed zero (going from positive
        // to zero/negative means the tilt is decreasing).
        // Require peakPitchRad > minimum to filter out noise on flat ground.
        boolean peakReached =
            pitchRate < BumpConstants.PITCH_PEAK_VELOCITY_THRESHOLD_RAD_PER_SEC
                && peakPitchRad > BumpConstants.PITCH_PEAK_MIN_RAD;

        if (peakReached) {
          transitionTo(BumpState.CRESTING);
        }
        break;

      case CRESTING:
        speedMs = BumpConstants.BUMP_SPEED_MS;

        // Descent confirmation: pitch has gone negative enough on the other
        // side of the bump.
        if (absPitch < -BumpConstants.PITCH_ENTRY_THRESHOLD_RAD) {
          transitionTo(BumpState.DESCENDING);
        }

        // Edge case: if the bump detection was very small (robot barely pitched) and
        // pitch has already returned near flat, skip straight to LEVELING.
        // if (absPitch < BumpConstants.PITCH_FLAT_THRESHOLD_RAD
        //     && peakPitchRad > BumpConstants.PITCH_PEAK_MIN_RAD) {
        //   transitionTo(BumpState.LEVELING);
        // }
        break;

      case DESCENDING:
        speedMs = BumpConstants.BUMP_SPEED_MS;

        // Rear wheels have cleared the ramp when pitch returns near flat.
        // Debouncer prevents early transition due to mid-ramp bounce.
        boolean nowFlat = absPitch < BumpConstants.PITCH_FLAT_THRESHOLD_RAD;
        if (flatDebouncer.calculate(nowFlat)) {
          transitionTo(BumpState.LEVELING);
        }
        break;

      case LEVELING:
        // Slow down slightly to confirm swerve stability before stopping.
        speedMs = BumpConstants.SETTLE_SPEED_MS;

        // The debouncer already fired to get us here, so just run for one
        // more SETTLE_DEBOUNCE_SECONDS worth of time for a smooth finish.
        // Use the flat debouncer again and reset it explicitly to restart the settle timer).
        // Simpler way would be to just stop after a tiny delay timer.
        // We transition to COMPLETE here and let WPI isFinished() end the command.
        transitionTo(BumpState.COMPLETE);
        break;

        // Falling cases are intended here, do not make mods without checking with Kaden first
      case COMPLETE:
      case TIMED_OUT:
      default:
        drive.stop();
        logState(pitch, pitchRate, 0.0);
        return;
    }

    // ----------------------------------Swerve Drive Commands------------------------------//
    // NOTE: This is assuming the robot should be aligned before calling this command.
    // THE USER MUST GET THIS RIGHT
    // If the robot is not aligned with the bump, add a heading controller here in a future
    // improvement.
    double vxFieldRelative = driveSign * speedMs;

    // Was originally going to use robotRelative speeds
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(vxFieldRelative, 0.0, 0.0), drive.getRotation());

    drive.runVelocity(fieldRelativeSpeeds);
    logState(pitch, pitchRate, vxFieldRelative);
  }

  @Override
  public boolean isFinished() {
    return state == BumpState.COMPLETE || state == BumpState.TIMED_OUT;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();

    if (interrupted) {
      Logger.recordOutput("CrossBump/State", "INTERRUPTED");
    } else {
      Logger.recordOutput("CrossBump/State", state.name());
    }

    Logger.recordOutput("CrossBump/ElapsedSeconds", totalTimer.get());
    totalTimer.stop();
    approachTimer.stop();
  }

  // ----------------------------------Internal Helper Methods------------------------------//

  private void transitionTo(BumpState next) {
    Logger.recordOutput("CrossBump/PreviousState", state.name());
    state = next;
    Logger.recordOutput("CrossBump/State", state.name());

    // Reset the flat debouncer whenever we leave DESCENDING so it starts
    // fresh if somehow re-entered.
    // If this triggers, honestly, ggs.
    if (next == BumpState.CRESTING || next == BumpState.APPROACHING) {
      flatDebouncer.calculate(false);
    }
  }

  /** NEW: Template logging format we should be using from now on */
  private void logState(double pitch, double pitchRate, double vx) {
    Logger.recordOutput("CrossBump/State", state.name());
    Logger.recordOutput("CrossBump/PitchRad", drive.getPitchPositionRad());
    Logger.recordOutput("CrossBump/PitchDeg", Math.toDegrees(pitch));
    Logger.recordOutput("CrossBump/PitchRateRadPerSec", pitchRate);
    Logger.recordOutput("CrossBump/PeakPitchDeg", Math.toDegrees(peakPitchRad));
    Logger.recordOutput("CrossBump/VxFieldRelative", vx);
    Logger.recordOutput("CrossBump/ElapsedSeconds", totalTimer.get());
  }

  // ----------------------------------Public AutoLogOutput Methods------------------------------//

  @AutoLogOutput(key = "CrossBump/ActiveState")
  public String getStateName() {
    return state.name();
  }
}
