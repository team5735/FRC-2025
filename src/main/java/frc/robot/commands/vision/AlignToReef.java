package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ReefAprilTagPositions;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Line;
import frc.robot.util.NTDoubleSection;
import frc.robot.util.ReefAlignment;
import frc.robot.util.TunablePIDController;

public class AlignToReef extends Command {
    private DrivetrainSubsystem drivetrain;

    private Pose2d alignmentTargetTag;
    private Line targetLine;

    public Line getTargetLine() {
        return targetLine;
    }

    private TunablePIDController omegaController = new TunablePIDController("AlignToReef_omega", 5, 1, 0);
    private TunablePIDController lineController = new TunablePIDController("AlignToReef_line", 5, 1, 0);

    private NTDoubleSection doubles = new NTDoubleSection(getName(), "omega", "deltaX", "deltaY");

    private static boolean infinite = true;

    private Supplier<Boolean> movingForward;

    private ReefAlignment whichBranch;

    private VisionSubsystem vision;

    /**
     * Positions the robot in order to score a coral.
     */
    public AlignToReef(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ReefAlignment whichBranch,
            Supplier<Boolean> movingForward) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.whichBranch = whichBranch;
        this.movingForward = movingForward;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        this.alignmentTargetTag = ReefAprilTagPositions
                .getClosestTag(drivetrain.getEstimatedPosition().getTranslation());
        this.targetLine = new Line(alignmentTargetTag, "AlignToReef").offsetBy(whichBranch.getParallel().in(Meters));

        omegaController.setup(alignmentTargetTag.getRotation().plus(Rotation2d.k180deg).getRadians(), 0.015);
        lineController.setup(0, .01); // we want to be 'at' the Line.

        vision.seedPigeon();

        SmartDashboard.putBoolean("aligning", true);
    }

    @Override
    public void execute() {
        Pose2d estimatedPosition = drivetrain.getEstimatedPosition();

        double omega = omegaController.calculate(estimatedPosition.getRotation().getRadians());

        double measurement = targetLine.getPIDMeasurement(estimatedPosition.getTranslation());
        double movementTowardsLine = lineController.calculate(measurement);
        Translation2d drivetrainMovement = targetLine.getVectorFrom(estimatedPosition.getTranslation())
                .times(-movementTowardsLine);

        doubles.set("omega", omega);
        doubles.set("deltaX", drivetrainMovement.getX());
        doubles.set("deltaY", drivetrainMovement.getY());

        if (lineController.getController().getError() < 0.05 && lineController.getController().getError() < 0.02) {
            drivetrainMovement = drivetrainMovement
                    .plus(targetLine.getVectorAlongLine().times(VisionConstants.ALONG_LINE_SPEED));
        }

        drivetrain.pidDrive(drivetrainMovement, omega);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("aligning", false);
    }

    private boolean atSetpoint() {
        return lineController.atSetpoint() && omegaController.atSetpoint();
    }

    @Override
    public boolean isFinished() {
        return !infinite && atSetpoint();
    }
}
