package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Arc;
import frc.robot.util.TunablePIDController;

public class DriveToArc extends Command {
    private DrivetrainSubsystem drivetrain;
    private Arc arc;

    private TunablePIDController pidX = new TunablePIDController("DriveToArc x");
    private TunablePIDController pidY = new TunablePIDController("DriveToArc y");
    private TunablePIDController pidTheta = new TunablePIDController("DriveToArc theta");

    public DriveToArc(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Translation2d robotPos = drivetrain.getEstimatedPosition().getTranslation();
        Pose2d targetPos = arc.nearestPoseOnArc(robotPos);
        pidX.setup(targetPos.getX());
        pidY.setup(targetPos.getY());
        pidTheta.setup(targetPos.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPos = drivetrain.getEstimatedPosition();
        Translation2d driveTrans = new Translation2d(pidX.calculate(robotPos.getX()), pidY.calculate(robotPos.getY()));
        double driveOmega = pidTheta.calculate(robotPos.getRotation().getRadians());
        drivetrain.pidDrive(driveTrans, driveOmega);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.pidDrive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return pidX.atSetpoint() && pidY.atSetpoint() && pidTheta.atSetpoint();
    }
}
