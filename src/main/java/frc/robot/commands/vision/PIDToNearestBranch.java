package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.constants.ReefAprilTagPositions;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.ReefAlignment;
import frc.robot.util.TunablePIDController;

public class PIDToNearestBranch extends Command {
    private Drivetrain drivetrain;

    private TunablePIDController xController = new TunablePIDController("PIDToNearestBranch_x");
    private TunablePIDController yController = new TunablePIDController("PIDToNearestBranch_y");
    private TunablePIDController omegaController = new TunablePIDController("PIDToNearestBranch_omega");

    private Supplier<ReefAlignment> alignment;

    public PIDToNearestBranch(Drivetrain drivetrain, Supplier<ReefAlignment> alignment) {
        this.drivetrain = drivetrain;
        this.alignment = alignment;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d targetPos = ReefAprilTagPositions
                .getClosest(drivetrain.getEstimatedPosition().getTranslation(),
                        ReefAprilTagPositions.SCORING_POSES.get(this.alignment.get()));

        Telemetry.field.getObject("closestBranch").setPose(targetPos);

        this.xController.setup(targetPos.getTranslation().getX(), 0.01);
        this.yController.setup(targetPos.getTranslation().getY(), 0.01);
        this.omegaController.setup(targetPos.getRotation().getDegrees(), 0.015);
        this.omegaController.getController().enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        double x = xController.calculate(drivetrain.getEstimatedPosition().getTranslation().getX());
        double y = yController.calculate(drivetrain.getEstimatedPosition().getTranslation().getY());
        double omega = omegaController.calculate(drivetrain.getEstimatedPosition().getRotation().getDegrees());

        double error = Math.sqrt(xController.getController().getError()
                * yController.getController().getError());
        if (error > VisionConstants.MAX_PID_ERROR) {
            drivetrain.setControl(drivetrain.brakeRequest);
            System.out.println("error too big: " + error);
            cancel();
            return;
        }

        drivetrain.pidDrive(x, y, omega);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint();
    }
}
