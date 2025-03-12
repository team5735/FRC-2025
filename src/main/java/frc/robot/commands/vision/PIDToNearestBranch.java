package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.constants.ReefAprilTagPositions;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.TunablePIDController;

public class PIDToNearestBranch extends Command {
    private DrivetrainSubsystem drivetrain;

    private TunablePIDController xController = new TunablePIDController("PIDToNearestBranch_x");
    private TunablePIDController yController = new TunablePIDController("PIDToNearestBranch_y");
    private TunablePIDController omegaController = new TunablePIDController("PIDToNearestBranch_omega");

    public PIDToNearestBranch(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d targetPos = ReefAprilTagPositions
                .getClosestScorePosition(drivetrain.getEstimatedPosition().getTranslation());

        Telemetry.field.getObject("closestBranch").setPose(targetPos);

        this.xController.setup(targetPos.getTranslation().getX(), 0.01);
        this.yController.setup(targetPos.getTranslation().getY(), 0.01);
        this.omegaController.setup(targetPos.getRotation().getDegrees(), 0.015);
    }

    @Override
    public void execute() {
        double x = xController.calculate(drivetrain.getEstimatedPosition().getTranslation().getX());
        double y = yController.calculate(drivetrain.getEstimatedPosition().getTranslation().getY());
        double omega = omegaController.calculate(drivetrain.getEstimatedPosition().getRotation().getDegrees());

        if (Math.sqrt(xController.getController().getError()
                * yController.getController().getError()) > VisionConstants.MAX_PID_ERROR) {
            drivetrain.setControl(drivetrain.brakeRequest);
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
