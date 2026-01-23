package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.List;

public class pathPlannerAutoAlign extends Command {
  // path constraints needs to be tuned
  public static PathConstraints defaultPathfindingConstraints =
      new PathConstraints(2.0, 3.5, Units.degreesToRadians(540), Units.degreesToRadians(720));
  PathPlannerPath path;
  List<Waypoint> waypoints;
  Pose2d targetPose;
  double endingVelocity;
  Rotation2d endRotation;
  Command runnable;

  public pathPlannerAutoAlign(Pose2d targetPose, double endingVelocity, Rotation2d endRotation) {
    this.targetPose = targetPose;
    // this.waypoints = PathPlannerPath.waypointsFromPoses(targetPose);
    this.endingVelocity = endingVelocity;
    this.endRotation = endRotation;
  }

  // this is a possible way of doing multiple paths so first work on one path -joshua
  // public void generatePath(){
  //     path = new PathPlannerPath(waypoints, defaultPathfindingConstraints, null, new
  // GoalEndState(0, null));
  //     path.preventFlipping = true;
  //     AutoBuilder.pathfindThenFollowPath(path, defaultPathfindingConstraints);
  // }

  // something to do is to check if the command should be scheduled inside execute or inside
  // initialize -joshua

  @Override
  public void execute() {
    runnable =
        AutoBuilder.pathfindToPose(targetPose, defaultPathfindingConstraints, endingVelocity);
    CommandScheduler.getInstance().schedule(runnable);
  }

  @Override
  public void end(boolean interrupted) {
    // runnable = false;
    CommandScheduler.getInstance().cancel(runnable);
  }

  @Override
  public boolean isFinished() {
    return runnable.isFinished();
  }
}
