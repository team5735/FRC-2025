package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class CoralConstants {
    // TODO FIND ACTUAL VOLTAGES FOR INTAKE
    // TODO FIND ACTUAL DELAY
    public static final double INTAKE_TOP_VOLTS = .75;
    public static final double INTAKE_BOTTOM_VOLTS = .75;

    public static final double TROUGH_TOP_VOLTS = 1.5;
    public static final double TROUGH_BOTTOM_VOLTS = 0.25;
    public static final Time TROUGH_TIMEOUT = Seconds.of(2);

    public static final double BRANCH_TOP_VOLTS = 1.0;
    public static final double BRANCH_BOTTOM_VOLTS = 1.0;
    public static final Time BRANCH_TIMEOUT = Seconds.of(2);

    public static final double FLIPPER_VOLTS = 2.0;
    public static final double UNFLIP_VOLTS = 1.5;

    public static final double FEEDER_VOLTS = 0.5;
    public static final double UNFEED_VOLTS = -1.0;

    public static final Time FEED_DELAY_SECONDS = Seconds.of(1.5);
    public static final Time L4_EJECTION_TIMEOUT = Seconds.of(1.5);
    public static final Time FLIPPER_RESET_DELAY = Seconds.of(1); // TODO test this value
    public static final Time FLIPPER_RESET_TIMOUT = Seconds.of(2); // TODO test this value
}
