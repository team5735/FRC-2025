package frc.robot.commands.vision;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ReefAprilTagPositions;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.ReefAlignment;

public class DriveToBranch extends Command {
    private DrivetrainSubsystem drivetrain;
    private Supplier<ReefAlignment> alignment;

    public DriveToBranch(DrivetrainSubsystem drivetrain, Supplier<ReefAlignment> alignment) {
        this.drivetrain = drivetrain;
        this.alignment = alignment;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        try {
            Pose2d tagPos = ReefAprilTagPositions.getClosestTag(drivetrain.getEstimatedPosition().getTranslation());
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                    new Pose2d(alignment.get().preAlignmentPosition(tagPos), tagPos.getRotation().unaryMinus()),
                    new Pose2d(alignment.get().scoringPosition(tagPos), tagPos.getRotation().unaryMinus()));
            PathConstraints constraints = DrivetrainSubsystem.CONSTANTS.getPathFollowConstraints();

            PathPlannerPath idealPath = new PathPlannerPath(waypoints, constraints, null,
                    new GoalEndState(0, tagPos.getRotation().unaryMinus()));

            new FollowPathCommand(
                    idealPath,
                    () -> drivetrain.getEstimatedPosition(),
                    () -> drivetrain.getChassisSpeeds(),
                    (speeds, ff) -> drivetrain.autoDriveRobotRelative(speeds),
                    new PPHolonomicDriveController(
                            DrivetrainSubsystem.CONSTANTS.getAutoPosConstants(),
                            DrivetrainSubsystem.CONSTANTS.getAutoRotConstants()),
                    DrivetrainSubsystem.CONSTANTS.getConfig(),
                    () -> false,
                    drivetrain).schedule();
        } catch (Exception e) {
            DriverStation.reportError("Path Follow Error: " + e.getMessage(), e.getStackTrace());
            System.out.println(e.getMessage());
        }
    }
}
