package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

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

    public enum OldLevel {
        BASE(() -> new State(BASE_HEIGHT.in(Units.Meters), 0), Color.kRed),
        PROCESSOR(() -> new State(PROCESS_HEIGHT.in(Units.Meters), 0), Color.kTurquoise),
        L1(() -> new State(L1_HEIGHT.in(Units.Meters), 0), Color.kOrange),
        L2(() -> new State(L2_HEIGHT.in(Units.Meters), 0), Color.kYellow),
        L3(() -> new State(L3_HEIGHT.in(Units.Meters), 0), Color.kGreen),
        L4(() -> new State(L4_HEIGHT.in(Units.Meters), 0), Color.kBlue),
        SMARTDASHBOARD(() -> new State(
                Inches.of(
                        SmartDashboard.getNumber("Elevator/HeightTargetInches", BASE_HEIGHT.in(Units.Inches)))
                        .in(Units.Meters),
                0), Color.kBisque);

        public final Supplier<State> stateSupplier;
        public final Color levelColor;

        private OldLevel(Supplier<State> stateSupplier, Color levelColor) {
            this.stateSupplier = stateSupplier;
            this.levelColor = levelColor;
        }
    }
}
