package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.TunablePIDController;

public class PIDToPose extends Command {
    private Supplier<Pose2d> poseSupplier;
    private Pose2d targetPose;

    private TunablePIDController pidX;
    private TunablePIDController pidY;
    private TunablePIDController pidTheta;

    private DrivetrainSubsystem drivetrain;

    /**
     * Create a PIDTOPose command with the given name.
     */
    public PIDToPose(DrivetrainSubsystem drivetrain, Supplier<Pose2d> poseSupplier, String name) {
        this.drivetrain = drivetrain;
        this.poseSupplier = poseSupplier;
        this.pidX = new TunablePIDController(name + " drive to pose x");
        this.pidY = new TunablePIDController(name + " drive to pose y");
        this.pidTheta = new TunablePIDController(name + " drive to pose theta");

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.targetPose = this.poseSupplier.get();
        this.pidX.setup(targetPose.getX());
        this.pidY.setup(targetPose.getY());
        this.pidTheta.setup(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getEstimatedPosition();
        double vx = this.pidX.calculate(robotPose.getX());
        double vy = this.pidY.calculate(robotPose.getY());
        double omega = this.pidTheta.calculate(robotPose.getRotation().getRadians());
        drivetrain.pidDrive(vx, vy, omega);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.pidDrive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return this.pidX.atSetpoint() && this.pidY.atSetpoint() && this.pidTheta.atSetpoint();
    }
}
