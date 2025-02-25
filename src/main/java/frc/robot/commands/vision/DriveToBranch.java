package frc.robot.commands.vision;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

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
            PathConstraints constraints = DrivetrainSubsystem.CONSTANTS.getPathFollowConstraints();
            
            AutoBuilder.pathfindToPose(
                new Pose2d(alignment.get().scoringPosition(tagPos), tagPos.getRotation().unaryMinus()), 
                constraints
            ).schedule();
        } catch (Exception e) {
            DriverStation.reportError("Path Follow Error: " + e.getMessage(), e.getStackTrace());
            System.out.println(e.getMessage());
        }
    }
}
