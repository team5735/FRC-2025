package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
    public static final int AVERAGING_WINDOW = 5;
    public static final Distance PATH_DIST_FROM_SCOREPOS = Meters.of(0.3);

    public static final double ALONG_LINE_SPEED = 1;

    public static final double MAX_PID_ERROR = 5;

    public static final Distance PID_DRIVE_THRESHOLD = Meters.of(0.5);

    public static final boolean IS_MT2 = true;

    public static final Distance RESET_PIGEON_DISTANCE = Meters.of(105);
    public static final int RESET_PIGEON_DELAY = 50;
}
