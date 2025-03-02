package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class CoralConstants {
    // TODO FIND ACTUAL VOLTAGES FOR INTAKE
    // TODO FIND ACTUAL DELAY
    public static final double INTAKE_TOP_VOLTS = 0.5;
    public static final double INTAKE_BOTTOM_VOLTS = 0.5;

    public static final double TROUGH_TOP_VOLTS = 1.5;
    public static final double TROUGH_BOTTOM_VOLTS = 0.25;

    public static final double BRANCH_TOP_VOLTS = 1.25;
    public static final double BRANCH_BOTTOM_VOLTS = 1.0;

    public static final double EJECTOR_VOLTS = 0.25;

    public static final double FEED_DELAY_SECONDS = 0;
    public static final Time L4_EJECTION_TIMEOUT = Seconds.of(1);
}
