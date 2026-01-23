package frc.pathplanner;

import org.junit.jupiter.api.Test;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.drivetrain.CompbotConstants;
import frc.robot.util.PathPlannerHelpers;

public class PathPlannerTest {
    @Test
    void linePathTest() throws Exception {
        CompbotConstants bot = new CompbotConstants();

        PathConstraints constraints = bot.getPathFollowConstraints();

        PathPlannerPath path = PathPlannerHelpers.createLinearPath(
                new Translation2d(0, 0),
                new Translation2d(1, 1),
                Rotation2d.fromDegrees(-45),
                new PathConstraints(3.0, 2.0, 360, 540));

        PathPlannerTrajectory trajectory = path.generateTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(0),
                bot.getConfig());

        // Sample points for plotting
        System.out.printf("x_meters, y_meters, rotation_degrees\n");
        for (var s : trajectory.getStates()) {
            Translation2d pos = s.pose.getTranslation();
            System.out.printf("%.2f, %.2f, %.2f\n",
                    pos.getX(),
                    pos.getY(),
                    s.pose.getRotation().getDegrees());
        }
    }
}
