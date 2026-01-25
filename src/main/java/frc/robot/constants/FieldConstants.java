package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.fieldmap.FieldAprilTag;

// This file provides positions, orientations, distances, and poses to field elements for
// the REBUILT 2026 competition playing field.
// These are based on the AndyMark field specifications

// all coordinates are in the wpiBlue coordinate frame, with (0,0,0) being blue alliance right corner
public class FieldConstants {
    private static double in2m(double inches) {return inches*2.54/100.0;}

    // gets the red alliance equivalent of the given blue field piece
    // the returned dimensions are STILL in wpiBlue coords
    public static Translation2d redElement(Translation2d blueElement){
        return new Translation2d(FIELD_LENGTH.in(Meters) - blueElement.getX(),
                                 FIELD_WIDTH.in(Meters) - blueElement.getY());
    }


    // dimensions of the arena
    public static final Distance FIELD_LENGTH = Meters.of(16.518); // extent along the x-axis
    public static final Distance FIELD_WIDTH = Meters.of(8.043);   // extent along the y-axis

    // from https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    // NOTE: we are only specifying blue elements.
    //       use the function redElement() to get the corresponding red alliance element.
    //       the coordinates for all elements are ALWAYS in the wpiBlue coordinate frame
    //       Red "RIGHT" elements are red alliance right ie viewed from the red alliance side
    //       So, when viewed from the blue side, the red element will be on the left
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(in2m(181.56), in2m(158.32));
    public static final Translation2d BLUE_OUTPOST_CENTER = new Translation2d(in2m(0), in2m(25.62));
    public static final Translation2d BLUE_TRENCH_RIGHT_CENTER = new Translation2d(in2m(181.56), in2m(24.97));
    public static final Translation2d BLUE_TRENCH_LEFT_CENTER = new Translation2d(in2m(181.56), in2m(FIELD_WIDTH.in(Inches)-24.97));
    public static final Distance      TRENCH_HEIGHT = Inches.of(22.25);

    public static final Translation2d BLUE_RAMP_RIGHT_CENTER = new Translation2d(in2m(181.56), in2m(24.97*2+12+73/2.0));
    public static final Translation2d BLUE_RAMP_LEFT_CENTER = new Translation2d(in2m(181.56), in2m(FIELD_WIDTH.in(Inches)-(24.97*2+12+73/2.0)));
    public static final Distance      RAMP_WIDTH = Inches.of(44); // extent along the x-axis of the field
    public static final Distance      RAMP_LENGTH = Inches.of(73); // extent along the y-axis of the field (note these are named opposite of FIELD_LENGTH and FIELD_WIDTH)

    // April tag section was created by reading the limelight fmap file and running this conversion script:
    // ./gradlew fieldfmap2constants --args "./src/main/java/frc/robot/constants/FRC2026_ANDYMARK.fmap"
    // The output is copy-pasted in here
    // (above FIELD_LENGTH and FIELD_WIDTH also come from this script)
    public static final Map<Integer, FieldAprilTag> APRIL_TAGS = Map.ofEntries(
            Map.entry(1,
                    new FieldAprilTag(1, Meters.of(0.165),
                            new Pose3d(new Translation3d(11.864, 7.411, 0.889),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(2,
                    new FieldAprilTag(2, Meters.of(0.165),
                            new Pose3d(new Translation3d(11.901, 4.625, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(90.0))))),
            Map.entry(3,
                    new FieldAprilTag(3, Meters.of(0.165),
                            new Pose3d(new Translation3d(11.298, 4.377, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(4,
                    new FieldAprilTag(4, Meters.of(0.165),
                            new Pose3d(new Translation3d(11.298, 4.021, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(5,
                    new FieldAprilTag(5, Meters.of(0.165),
                            new Pose3d(new Translation3d(11.901, 3.418, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-90.0))))),
            Map.entry(6,
                    new FieldAprilTag(6, Meters.of(0.165),
                            new Pose3d(new Translation3d(11.864, 0.631, 0.889),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(7,
                    new FieldAprilTag(7, Meters.of(0.165),
                            new Pose3d(new Translation3d(11.939, 0.631, 0.889),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(8,
                    new FieldAprilTag(8, Meters.of(0.165),
                            new Pose3d(new Translation3d(12.257, 3.418, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-90.0))))),
            Map.entry(9,
                    new FieldAprilTag(9, Meters.of(0.165),
                            new Pose3d(new Translation3d(12.505, 3.666, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(10,
                    new FieldAprilTag(10, Meters.of(0.165),
                            new Pose3d(new Translation3d(12.505, 4.021, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(11,
                    new FieldAprilTag(11, Meters.of(0.165),
                            new Pose3d(new Translation3d(12.257, 4.625, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(90.0))))),
            Map.entry(12,
                    new FieldAprilTag(12, Meters.of(0.165),
                            new Pose3d(new Translation3d(11.939, 7.411, 0.889),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(13,
                    new FieldAprilTag(13, Meters.of(0.165),
                            new Pose3d(new Translation3d(16.499, 7.392, 0.552),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(14,
                    new FieldAprilTag(14, Meters.of(0.165),
                            new Pose3d(new Translation3d(16.499, 6.960, 0.552),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(15,
                    new FieldAprilTag(15, Meters.of(0.165),
                            new Pose3d(new Translation3d(16.499, 4.312, 0.552),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(16,
                    new FieldAprilTag(16, Meters.of(0.165),
                            new Pose3d(new Translation3d(16.499, 3.881, 0.552),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(17,
                    new FieldAprilTag(17, Meters.of(0.165),
                            new Pose3d(new Translation3d(4.649, 0.631, 0.889),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(18,
                    new FieldAprilTag(18, Meters.of(0.165),
                            new Pose3d(new Translation3d(4.612, 3.418, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-90.0))))),
            Map.entry(19,
                    new FieldAprilTag(19, Meters.of(0.165),
                            new Pose3d(new Translation3d(5.215, 3.666, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(20,
                    new FieldAprilTag(20, Meters.of(0.165),
                            new Pose3d(new Translation3d(5.215, 4.021, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(21,
                    new FieldAprilTag(21, Meters.of(0.165),
                            new Pose3d(new Translation3d(4.612, 4.625, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(90.0))))),
            Map.entry(22,
                    new FieldAprilTag(22, Meters.of(0.165),
                            new Pose3d(new Translation3d(4.649, 7.411, 0.889),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(23,
                    new FieldAprilTag(23, Meters.of(0.165),
                            new Pose3d(new Translation3d(4.574, 7.411, 0.889),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(24,
                    new FieldAprilTag(24, Meters.of(0.165),
                            new Pose3d(new Translation3d(4.256, 4.625, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(90.0))))),
            Map.entry(25,
                    new FieldAprilTag(25, Meters.of(0.165),
                            new Pose3d(new Translation3d(4.008, 4.377, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(26,
                    new FieldAprilTag(26, Meters.of(0.165),
                            new Pose3d(new Translation3d(4.008, 4.021, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(27,
                    new FieldAprilTag(27, Meters.of(0.165),
                            new Pose3d(new Translation3d(4.256, 3.418, 1.124),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-90.0))))),
            Map.entry(28,
                    new FieldAprilTag(28, Meters.of(0.165),
                            new Pose3d(new Translation3d(4.574, 0.631, 0.889),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(180.0))))),
            Map.entry(29,
                    new FieldAprilTag(29, Meters.of(0.165),
                            new Pose3d(new Translation3d(0.014, 0.651, 0.552),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(30,
                    new FieldAprilTag(30, Meters.of(0.165),
                            new Pose3d(new Translation3d(0.014, 1.083, 0.552),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(31,
                    new FieldAprilTag(31, Meters.of(0.165),
                            new Pose3d(new Translation3d(0.014, 3.730, 0.552),
                                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))),
            Map.entry(32, new FieldAprilTag(32, Meters.of(0.165), new Pose3d(new Translation3d(0.014, 4.162, 0.552),
                    new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0))))));
}
