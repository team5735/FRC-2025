package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.ReefAlignment;

public class ReefAprilTagPositions {
    /**
     * Tags around the blue alliance reef. Field-space, centered at field origin
     * (unlike the rest of the robot code .-.)
     *
     * <p>
     * Here's a diagram of the reefs with their tags.
     * The 'x' in the middle is the field space origin. The arrow is θ=0
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
    public static final Pose2d BLUE_TAGS[] = new Pose2d[] {
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

    // These two numbers aren't factor-of-two related because of the net in the
    // middle of the field.
    private static final double APRILTAG_TEAM_OFFSET = 8.570;
    private static final double FIELD_LENGTH = 17.5483;
    private static final double FIELD_WIDTH = 8.0519;

    public static final Pose2d TAGS[] = new Pose2d[BLUE_TAGS.length * 2];

    static {
        Pose2d[] redTags = new Pose2d[BLUE_TAGS.length];
        for (int i = 0; i < BLUE_TAGS.length; i++) {
            redTags[i] = new Pose2d(BLUE_TAGS[i].getTranslation().plus(
                    new Translation2d(APRILTAG_TEAM_OFFSET, 0)), BLUE_TAGS[i].getRotation());
        }
        System.arraycopy(BLUE_TAGS, 0, TAGS, 0, BLUE_TAGS.length);
        System.arraycopy(redTags, 0, TAGS, BLUE_TAGS.length, redTags.length);
    }

    private static Pose2d createScoringPoseWithOffset(Pose2d pose, ReefAlignment offset) {
        return new Pose2d(
                pose.getTranslation()
                        .plus(new Translation2d(offset.getParallel().in(Meters),
                                pose.getRotation().plus(Rotation2d.kCCW_Pi_2))),
                pose.getRotation().plus(Rotation2d.kPi)).transformBy(
                        new Transform2d(
                                new Translation2d(
                                        -Drivetrain.CONSTANTS.getPigeonToRobotFront().minus(Centimeters.of(5))
                                                .in(Meters),
                                        0),
                                Rotation2d.kZero));
    }

    public static final HashMap<ReefAlignment, Pose2d[]> SCORING_POSES = new HashMap<>();
    static {
        for (ReefAlignment align : ReefAlignment.values()) {
            Pose2d[] poses = new Pose2d[TAGS.length];
            for (int i = 0; i < TAGS.length; i++) {
                poses[i] = createScoringPoseWithOffset(TAGS[i], align);
            }
            SCORING_POSES.put(align, poses);
        }
    }

    private static Pose2d fieldSpaceToBlueAllianceSpace(Pose2d in) {
        Translation2d trans = new Translation2d(in.getTranslation().getX() + FIELD_LENGTH / 2,
                in.getTranslation().getY() + FIELD_WIDTH / 2);
        return new Pose2d(trans, in.getRotation());
    }

    private static final StructPublisher<Pose2d> closestPosePublisher = NetworkTableInstance.getDefault()
            .getTable("sections").getSubTable("closestTag").getStructTopic("pos", Pose2d.struct).publish();

    public static Pose2d getClosest(Translation2d position, Pose2d[] poses) {
        double closestDistanceSoFar = Double.MAX_VALUE;
        Pose2d best = null;
        for (Pose2d tag : poses) {
            double dist = tag.getTranslation().getDistance(position);
            if (dist < closestDistanceSoFar) {
                best = tag;
                closestDistanceSoFar = dist;
            }
        }
        closestPosePublisher.accept(best);
        return best;
    }

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
        return getClosest(position, TAGS);
    }

    public static Pose2d getClosestScorePosition(Translation2d position) {
        Pose2d[] allCoralPositions = new Pose2d[TAGS.length * 2];
        System.arraycopy(SCORING_POSES.get(ReefAlignment.LEFT), 0, allCoralPositions, 0, TAGS.length);
        System.arraycopy(SCORING_POSES.get(ReefAlignment.RIGHT), 0, allCoralPositions, TAGS.length, TAGS.length);
        return getClosest(position, allCoralPositions);
    }

    public static double getScoreDistance(Drivetrain drivetrain) {
        Pose2d drivetrainPos = drivetrain.getEstimatedPosition();
        Pose2d scorePos = getClosestScorePosition(drivetrainPos.getTranslation());
        return scorePos.getTranslation().getDistance(drivetrainPos.getTranslation());
    }

    public static final Distance DISTANCE_BETWEEN_BRANCHES = Inches.of(12.94); // exact from field diagram
}
