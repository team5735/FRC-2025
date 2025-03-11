package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.constants.ReefAprilTagPositions;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.TunablePIDController;

public class PIDToNearestBranch extends Command {

    private DrivetrainSubsystem drivetrain;
    private VisionSubsystem vision;
    private Supplier<Translation2d> targetSupplier;

    private TunablePIDController xController;
    private TunablePIDController omegaController;
    private TunablePIDController yController;

    public PIDToNearestBranch(DrivetrainSubsystem drivetrain, VisionSubsystem vision,
            Supplier<Translation2d> targetSupplier) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetSupplier = targetSupplier;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        Pose2d targetPos = ReefAprilTagPositions
                .getClosestScorePosition(drivetrain.getEstimatedPosition().getTranslation());

        Telemetry.field.getObject("closestBranch").setPose(targetPos);

        this.xController = new TunablePIDController("PIDToNearestBranch_x");
        this.yController = new TunablePIDController("PIDToNearestBranch_y");
        this.omegaController = new TunablePIDController("PIDToNearestBranch_omega");

        this.xController.setup(targetPos.getTranslation().getX());
        this.yController.setup(targetPos.getTranslation().getY());
        this.omegaController.setup(targetPos.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        double x = xController.calculate(drivetrain.getEstimatedPosition().getTranslation().getX());
        double y = yController.calculate(drivetrain.getEstimatedPosition().getTranslation().getY());
        double omega = omegaController.calculate(drivetrain.getEstimatedPosition().getRotation().getDegrees());
        drivetrain.pidDrive(x, y, omega);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint();
    }
}
