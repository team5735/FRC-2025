package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.Meters;

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
import frc.robot.util.NTBooleanSection;
import frc.robot.util.NTDoubleSection;
import frc.robot.util.TunablePIDController;

public class AlignToReef extends Command {
    private DrivetrainSubsystem drivetrain;

    private Pose2d alignmentTargetPos;
    private Line alignmentTargetLine;

    public Line getAlignmentTargetLine() {
        return alignmentTargetLine;
    }

    private TunablePIDController angleController = new TunablePIDController("AlignToReef_omega", 5, 1, 0);
    private TunablePIDController lineController = new TunablePIDController("AlignToReef_line", 5, 1, 0);

    private NTDoubleSection doubles = new NTDoubleSection(getName(), "omega", "deltaX", "deltaY");
    private NTBooleanSection booleans = new NTBooleanSection(getName(), "aligning");

    private static boolean infinite = true;

    private VisionSubsystem vision;

    /**
     * Positions the robot in order to score a coral.
     */
    public AlignToReef(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        this.alignmentTargetPos = ReefAprilTagPositions
                .getClosestScorePosition(drivetrain.getEstimatedPosition().getTranslation());
        this.alignmentTargetLine = new Line(
                this.alignmentTargetPos.getTranslation(),
                this.alignmentTargetPos.getRotation().plus(Rotation2d.k180deg),
                "alignToReef");

        angleController.setup(alignmentTargetPos.getRotation().plus(Rotation2d.k180deg).getRadians(), 0.015);
        lineController.setup(0, .01); // we want to be 'at' the Line.

        vision.seedPigeon();

        booleans.set("aligning", true);
    }

    @Override
    public void execute() {
        Pose2d estimatedPosition = drivetrain.getEstimatedPosition();

        double omega = angleController.calculate(estimatedPosition.getRotation().getRadians());

        double measurement = alignmentTargetLine.getPIDMeasurement(estimatedPosition.getTranslation()).in(Meters);
        double movementTowardsLine = lineController.calculate(measurement);
        Translation2d drivetrainMovement = alignmentTargetLine.getVectorFrom(estimatedPosition.getTranslation())
                .times(movementTowardsLine);

        doubles.set("omega", omega);
        doubles.set("deltaX", drivetrainMovement.getX());
        doubles.set("deltaY", drivetrainMovement.getY());

        if (lineController.atSetpoint()) {
            drivetrainMovement = drivetrainMovement
                    .plus(alignmentTargetLine.getVectorAlongLine().times(VisionConstants.ALONG_LINE_SPEED));
        }

        drivetrain.pidDrive(drivetrainMovement, omega);
    }

    @Override
    public void end(boolean interrupted) {
        booleans.set("aligning", false);
    }

    private boolean atSetpoint() {
        return lineController.atSetpoint() && angleController.atSetpoint();
    }

    @Override
    public boolean isFinished() {
        return !infinite && atSetpoint();
    }
}
