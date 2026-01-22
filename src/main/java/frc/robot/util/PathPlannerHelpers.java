package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathPlannerHelpers{
    /**
     * Creates a path that is a line from the start point to the end point
     *
     * @param startPoint      The start of the path in field coordinates
     * @param endPoint        The end of the path in field coordinates
     * @param endRobotHeading The direction the robot should be facing at the end of the path
     * @param constraints     The constraints to use for the path (eg DrivetrainSubsystem.CONSTANTS.getPathFollowConstraints())
     * @return The path
    */    
    public static PathPlannerPath createLinearPath(Translation2d startPoint,
                                                   Translation2d endPoint,
                                                   Rotation2d endRobotHeading,
                                                   PathConstraints constraints) {
        // the path heading is different than the robot heading. The path heading
        // points in the direction of travel. This ensures the resulting path (a Bezier curve) is well formed
        // without swoops and small loops in it
        // for a path that is simply a line, the direction of travel is constant: heading toward the end point
        Rotation2d pathHeading = endPoint.minus(startPoint).getAngle();
        Pose2d startPose = new Pose2d(startPoint, pathHeading);
        Pose2d endPose = new Pose2d(endPoint, pathHeading);
        //endPose = new Pose2d(endPoint, endRobotHeading); // this will create a swooping end trajectory
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);
        PathPlannerPath path = new PathPlannerPath(waypoints,
                                                   constraints,
                                                   null, // Ideal starting state can be null for on-the-fly paths
                                                   new GoalEndState(0, endRobotHeading));

        // Create that path exactly. Don't flip it to be the red alliance.
        // That can be handled by whoever calls this
        path.preventFlipping = true;

        return path;
    }
}