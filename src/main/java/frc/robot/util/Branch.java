package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.drivetrain.DrivetrainConstants;

public enum Branch {
    LEFT(DrivetrainConstants.ROBOT_TOTAL_LENGTH.div(2), Feet.of(0)), // TODO find actual parallel alignment
    RIGHT(DrivetrainConstants.ROBOT_TOTAL_LENGTH.div(2), Feet.of(0)),  // TODO find actual parallel alignment
    NEITHER(DrivetrainConstants.ROBOT_TOTAL_LENGTH.div(2), Meters.of(0));

    public final Distance perpendicular, parallel;

    Branch(Distance perpendicular, Distance parallel){
        this.perpendicular = perpendicular;
        this.parallel = parallel;
    }

    public Translation2d scoringPosition(Pose2d tagPos){
        Rotation2d theta = tagPos.getRotation();

        Translation2d offset = new Translation2d(perpendicular, parallel).rotateBy(theta);

        return tagPos.getTranslation().plus(offset);
    }
}
