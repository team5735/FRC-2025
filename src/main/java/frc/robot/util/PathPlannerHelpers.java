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
     * @param nextWaypoint    If we're chaining paths together, this is the next point we will follow
     *                        Example: suppose we want a path to points a-b-c, where they are not all collinear. 
     *                                 Without nextWaypoint, we can create path a-b, and then another path b-c with
     *                                 2 calls to this function. But this will create a discontinuity at point b.
     *                                 Instead, when creating path a-b, we can pass in point c as the next waypoint
     *                                 and the curve for a-b will be positioned properly to chain smoothly with b-c
     * @param constraints     The constraints to use for the path (eg DrivetrainSubsystem.CONSTANTS.getPathFollowConstraints())
     * @return The path
    */    
    public static PathPlannerPath createLinearPath(Translation2d startPoint,
                                                   Translation2d endPoint,
                                                   Rotation2d endRobotHeading,
                                                   Translation2d nextWaypoint,
                                                   PathConstraints constraints) {
        // the path heading is different than the robot heading. The path heading
        // points in the direction of travel. This ensures the resulting path (a Bezier curve) is well formed
        // without swoops and small loops in it
        // for a path that is simply a line, the direction of travel is constant: heading toward the end point
        Rotation2d pathHeading = endPoint.minus(startPoint).getAngle();
        Pose2d startPose = new Pose2d(startPoint, pathHeading);
        Pose2d endPose = new Pose2d(endPoint, pathHeading);
        //endPose = new Pose2d(endPoint, endRobotHeading); // this will create a swooping end trajectory

        if (nextWaypoint != null){
            endPose = new Pose2d(endPoint, nextWaypoint.minus(endPoint).getAngle());
        }

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

    /**
     * Creates a path that is a line from the start point to the end point
     * with no need to chain subsequent paths together
     */
    public static PathPlannerPath createLinearPath(Translation2d startPoint,
                                                   Translation2d endPoint,
                                                   Rotation2d endRobotHeading,
                                                   PathConstraints constraints){
    
        return createLinearPath(startPoint, endPoint, endRobotHeading, null, constraints);
    }
}