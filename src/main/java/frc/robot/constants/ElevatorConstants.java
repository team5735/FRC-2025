package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorConstants {
    public static final int KP = 0;
    public static final int KI = 0;
    public static final int KD = 0;
    public static final int KS = 0;
    public static final int KG = 0;
    public static final int KV = 0;
    public static final int KA = 0;
    public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(0);
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
    public static final double ROTATIONS_TO_FEET = 0;
    public static final Distance BASE_HEIGHT = Feet.of(0);
    public static final Distance L1_HEIGHT = Feet.of(0);
    public static final Distance L2_HEIGHT = Feet.of(0);
    public static final Distance L3_HEIGHT = Feet.of(0);
    public static final Distance L4_HEIGHT = Feet.of(0);
    public static final Distance PROCESS_HEIGHT = Feet.of(0);

    public enum Level{
        BASE(() -> new State(BASE_HEIGHT.in(Units.Meters), 0)),
        PROCESSOR(() -> new State(PROCESS_HEIGHT.in(Units.Meters), 0)),
        L1(() -> new State(L1_HEIGHT.in(Units.Meters), 0)),
        L2(() -> new State(L2_HEIGHT.in(Units.Meters), 0)),
        L3(() -> new State(L3_HEIGHT.in(Units.Meters), 0)),
        L4(() -> new State(BASE_HEIGHT.in(Units.Meters), 0)),
        SMARTDASHBOARD(() -> new State(
            Feet.of(
                SmartDashboard.getNumber("Elevator/HeightTargetFeet", BASE_HEIGHT.in(Units.Feet))).in(Units.Meters
            ), 
            0
        ));

        public final Supplier<State> stateSupplier;

        private Level(Supplier<State> stateSupplier){
            this.stateSupplier = stateSupplier;
        }
    }
}