package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Telemetry;
import frc.robot.constants.ReefAprilTagPositions;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.ReefAlignment;

public class DriveToBranch extends Command {
    private DrivetrainSubsystem drivetrain;
    private Command storedCommand = Commands.none();
    private Watchdog watchdog = new Watchdog(1, () -> {
    });

    public DriveToBranch(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    private Pose2d invertAngleOfPose(Pose2d in) {
        return new Pose2d(in.getTranslation(), in.getRotation().plus(Rotation2d.k180deg));
    }

    @Override
    public void initialize() {
        try {
            watchdog.reset();
            Pose2d tagPos = ReefAprilTagPositions
                    .getClosestScorePosition(drivetrain.getEstimatedPosition().getTranslation());
            Telemetry.field.getObject("targetPose").setPose(tagPos);
            watchdog.addEpoch("tagPos retrieved");
            PathConstraints constraints = DrivetrainSubsystem.CONSTANTS.getPathFollowConstraints();

            Pose2d lineUpPathOtherNode = new Pose2d(
                    tagPos.getTranslation()
                            .plus(new Translation2d(VisionConstants.PATH_DIST_FROM_SCOREPOS.in(Meters),
                                    tagPos.getRotation().plus(Rotation2d.k180deg))),
                    tagPos.getRotation());
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(lineUpPathOtherNode,
                    invertAngleOfPose(tagPos));
            watchdog.addEpoch("create waypoints");

            PathPlannerPath path = new PathPlannerPath(waypoints, constraints,
                    new IdealStartingState(0.5, tagPos.getRotation()),
                    new GoalEndState(0, tagPos.getRotation()));
            path.preventFlipping = true;
            watchdog.addEpoch("create path");

            storedCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
            watchdog.addEpoch("made new storedCommand");
            storedCommand.schedule();
            watchdog.printEpochs();
        } catch (Exception e) {
            DriverStation.reportError("Path Follow Error: " + e.getMessage(), e.getStackTrace());
            System.out.println(e.getMessage());
        }
    }
}
