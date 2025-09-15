package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
    public static final int AVERAGING_WINDOW = 5;
    public static final Distance PATH_DIST_FROM_SCOREPOS = Meters.of(0.3);

    // If the line error in AlignToReef is less than this and B is held, go
    // forwards.
    public static final double MAX_LINE_ERROR = 0.02;
    public static final double ALONG_LINE_SPEED = 1;
}
