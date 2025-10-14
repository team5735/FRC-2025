package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorConstants {
    public static final double KP = 3;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0.0925;
    public static final double KG = 0.1925;
    public static final double KV = 5.74;
    public static final double KA = 3.1202;

    public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1.5);
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(2.25);
    public static final double INCHES_PER_ENCODER_COUNTS = 0.7465;

    public static final Distance BASE_HEIGHT = Inches.of(0);
    public static final Distance L1_HEIGHT = Inches.of(5);
    public static final Distance L2_HEIGHT = Inches.of(13);
    public static final Distance L3_HEIGHT = Inches.of(29);
    public static final Distance L4_HEIGHT = Inches.of(53.5);
    public static final Distance PROCESS_HEIGHT = Feet.of(0);
    public static final Distance MAX_HEIGHT = L4_HEIGHT;

    public static final Distance AT_LEVEL_THRESHOLD = Millimeters.of(5);

    public enum Height {
        BASE(Inches.of(0)),
        L1(Inches.of(5)),
        L2(Inches.of(13)),
        L3(Inches.of(29)),
        L4(Inches.of(53.5)),
        PROCESS(Inches.of(0));

        public final Distance height;

        private Height(Distance height) {
            this.height = height;
        }
    }
}
