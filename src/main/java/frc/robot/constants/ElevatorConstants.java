package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;

public class ElevatorConstants {
    public static final double KP = 3;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0.0925;
    public static final double KG = 0.1925;
    public static final double KV = 5.5;
    public static final double KA = 3.1202;
    public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1);
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(1.25);
    public static final double INCHES_PER_ROTATIONS = 0.80474;
    public static final Distance BASE_HEIGHT = Inches.of(0);
    public static final Distance L1_HEIGHT = Inches.of(10);
    public static final Distance L2_HEIGHT = Inches.of(18.25);
    public static final Distance L3_HEIGHT = Inches.of(24.25);
    public static final Distance L4_HEIGHT = Inches.of(0);
    public static final Distance PROCESS_HEIGHT = Feet.of(0);
    public static final Distance MAX_HEIGHT = Inches.of(53.25);

    public enum Level {
        BASE(() -> new State(BASE_HEIGHT.in(Units.Meters), 0), Color.kAliceBlue),
        PROCESSOR(() -> new State(PROCESS_HEIGHT.in(Units.Meters), 0), Color.kAntiqueWhite),
        L1(() -> new State(L1_HEIGHT.in(Units.Meters), 0), Color.kAqua),
        L2(() -> new State(L2_HEIGHT.in(Units.Meters), 0), Color.kAquamarine),
        L3(() -> new State(L3_HEIGHT.in(Units.Meters), 0), Color.kAzure),
        L4(() -> new State(L4_HEIGHT.in(Units.Meters), 0), Color.kBeige),
        SMARTDASHBOARD(() -> new State(
                Feet.of(
                        SmartDashboard.getNumber("Elevator/HeightTargetFeet", BASE_HEIGHT.in(Units.Feet)))
                        .in(Units.Meters),
                0), Color.kBisque);
        // IDLE(() -> new State(RobotContainer.elevator.getPosition().in(Units.Meters),
        // 0), Color.kOrange);

        public final Supplier<State> stateSupplier;
        public final Color levelColor;

        private Level(Supplier<State> stateSupplier, Color levelColor) {
            this.stateSupplier = stateSupplier;
            this.levelColor = levelColor;
        }
    }
}