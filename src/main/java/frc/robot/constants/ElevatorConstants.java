package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

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
}