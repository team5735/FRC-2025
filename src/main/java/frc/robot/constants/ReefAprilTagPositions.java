package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;

public class ReefAprilTagPositions {
    /**
     * Tags around the blue alliance reef. Field-space, centered at field origin
     * (unlike the rest of the robot code .-.)
     *
     * <p>
     * Here's a diagram of the reefs with their tags.
     * The 'x' in the middle is the field origin. The arrow is θ=0
     * The 'y' in the bottom left is the blue alliance field origin.
     * In reality, each reef is a hexagon (so no __ sides), but I couldn't make that
     * look nice with ASCII.
     * <code>
     * Blue alliance             Red alliance
     * v                                    v
     * / ***************************************** \
     * |         __                     __         |
     * |      19/  \20                9/  \8       |
     * |     18|    |21      x →    10|    |7      |
     * |      17\__/22               11\__/6       |
     * |                                           |
     * \y***************************************** /
     * </code>
     */
    public static final Pose2d TAGS[] = new Pose2d[] {
            // 17
            fieldSpaceToBlueAllianceSpace(new Pose2d(
                    new Translation2d(-4.700446, -0.719482),
                    Rotation2d.fromDegrees(-120))),
            // 18
            fieldSpaceToBlueAllianceSpace(new Pose2d(
                    new Translation2d(-5.116498, 0),
                    Rotation2d.fromDegrees(180))),
            // 19
            fieldSpaceToBlueAllianceSpace(new Pose2d(
                    new Translation2d(-4.700446, 0.719482),
                    Rotation2d.fromDegrees(120))),
            // 20
            fieldSpaceToBlueAllianceSpace(new Pose2d(
                    new Translation2d(-3.869358, 0.719482),
                    Rotation2d.fromDegrees(60))),
            // 21
            fieldSpaceToBlueAllianceSpace(new Pose2d(
                    new Translation2d(-3.453306, 0),
                    Rotation2d.fromDegrees(0))),
            // 22
            fieldSpaceToBlueAllianceSpace(new Pose2d(
                    new Translation2d(-3.869358, -0.719482),
                    Rotation2d.fromDegrees(-60))),
    };

    private static final double FIELD_LENGTH = 17.5483;
    private static final double FIELD_WIDTH = 8.0519;

    private static Pose2d fieldSpaceToBlueAllianceSpace(Pose2d in) {
        Translation2d trans = new Translation2d(in.getTranslation().getX() + FIELD_LENGTH / 2,
                in.getTranslation().getY() + FIELD_WIDTH / 2);
        return new Pose2d(trans, in.getRotation());
    }

    private static final StructPublisher<Pose2d> closestPosePublisher = NetworkTableInstance.getDefault()
            .getTable("sections").getSubTable("closestTag").getStructTopic("pos", Pose2d.struct).publish();

    /**
     * Returns the pose of the tag closest to the given position.
     *
     * @param position The position in blue alliance field space.
     */
    public static Pose2d getClosestTag(Translation2d position) {
        if (position.getX() > FIELD_LENGTH / 2) {
            position = new Translation2d(position.getX() - FIELD_LENGTH / 2, position.getY());
            System.out.println("shifted");
        }
        System.out.println("searching for closest tag to (x, y) = (" + position.getX() + ", " + position.getY() + ")");

        double closestDistanceSoFar = Double.MAX_VALUE;
        Pose2d best = null;
        for (Pose2d tag : TAGS) {
            double dist = tag.getTranslation().getDistance(position);
            if (dist < closestDistanceSoFar) {
                best = tag;
                closestDistanceSoFar = dist;
            }
        }
        closestPosePublisher.accept(best);
        return best;
    }

    public static final Distance DISTANCE_BETWEEN_BRANCHES = Inches.of(12.94); // exact from field diagram
}
