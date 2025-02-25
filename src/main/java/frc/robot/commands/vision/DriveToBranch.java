package frc.robot.commands.vision;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ReefAprilTagPositions;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.ReefAlignment;

public class DriveToBranch extends Command {
    private DrivetrainSubsystem drivetrain;
    private Supplier<ReefAlignment> alignment;
    private Command storedCommand = Commands.none();

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
            ).withDeadline(this).schedule();
        } catch (Exception e) {
            DriverStation.reportError("Path Follow Error: " + e.getMessage(), e.getStackTrace());
            System.out.println(e.getMessage());
        }
    }
}
