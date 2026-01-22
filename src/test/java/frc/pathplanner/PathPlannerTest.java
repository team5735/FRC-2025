import org.junit.jupiter.api.Test;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.drivetrain.CompbotConstants;

public class PathPlannerTest {
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

    @Test
    void linePathTest() throws Exception {
        CompbotConstants bot = new CompbotConstants();

        PathConstraints constraints = bot.getPathFollowConstraints();

        PathPlannerPath path = createLinearPath(
            new Translation2d(0,0), 
            new Translation2d(1,1), 
            Rotation2d.fromDegrees(135), 
            new PathConstraints(3.0, 2.0, 360, 540));
            
        PathPlannerTrajectory trajectory = path.generateTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(0),bot.getConfig());
        
        // Sample points for plotting
        for (var s : trajectory.getStates()) {
            Translation2d pos = s.pose.getTranslation();
            System.out.printf("%.2f, %.2f, %.2f\n",
                pos.getX(),
                pos.getY(),
                s.pose.getRotation().getDegrees()
            );
        }
    }
}