package frc.robot.util;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.DrivetrainSubsystem;

public enum ReefAlignment {
    LEFT(Inches.of(4)),
    RIGHT(Inches.of(17)),
    ALGAE(Meters.of(0));

    private Distance parallel;

    public Distance getParallel() {
        return parallel;
    }

    ReefAlignment(Distance parallel) {
        this.parallel = parallel;
    }

    public Translation2d scoringPosition(Pose2d tagPos) {
        Rotation2d theta = tagPos.getRotation();

        Translation2d offset = new Translation2d(DrivetrainSubsystem.CONSTANTS.getPigeonToRobotFront(), parallel)
                .rotateBy(theta);

        return tagPos.getTranslation().plus(offset);
    }

    public Translation2d preAlignmentPosition(Pose2d tagPos) {
        Rotation2d theta = tagPos.getRotation();

        Translation2d offset = new Translation2d(Feet.of(2), Meters.of(0))
                .rotateBy(theta);

        return tagPos.getTranslation().plus(offset);

    }
}
