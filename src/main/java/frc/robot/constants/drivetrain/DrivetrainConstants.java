package frc.robot.constants.drivetrain;

import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    public static final double ROTATION_KP = 0.014787;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0;
    public static final double ROTATION_KS = 0.08504;
    public static final double ROTATION_KV = 0.013474;
    public static final double ROTATION_KA = 0.0050825;

    public static final double DEADBAND = 0.15;
    public static final double SPEED_MPS = 2;
    public static final double SPIN_RPS = 0.25;
    public static final double ROBOT_MASS_KG = 25.35;
    public static final double MAX_WHEEL_DISTANCE_M = Units.inchesToMeters(25);
    public static final double ROBOT_MOI_KGxMxM = 
            ROBOT_MASS_KG * MAX_WHEEL_DISTANCE_M / 2 * ROTATION_KA / CompbotTunerConstants.driveGains.kA; // Formula for approx. MoI
    public static final double COEFFICIENT_OF_FRICTION = 1;
}
