package frc.robot.constants.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    public static final PIDConstants AUTO_POS_CONSTANTS = new PIDConstants(10, 0);
    public static final PIDConstants AUTO_ROT_CONSTANTS = new PIDConstants(15, 0);

    public static final double SPIN_KP = 0.014787;
    public static final double SPIN_KI = 0;
    public static final double SPIN_KD = 0;
    public static final double SPIN_KS = 0.08504;
    public static final double SPIN_KV = 0.013474;
    public static final double SPIN_KA = 0.0050825;

    public static final double DEADBAND = 0.15;
    public static final double SPEED_MPS = 4;
    public static final double SPIN_RPS = 0.25;
    public static final double ROBOT_MASS_KG = 25.35;
    public static final double MAX_WHEEL_DISTANCE_M = Units.inchesToMeters(25);
    public static final double ROBOT_MOI_KGxMxM = ROBOT_MASS_KG * MAX_WHEEL_DISTANCE_M / 2 * SPIN_KA
            / CompbotTunerConstants.DRIVE_GAINS.kA; // Formula for approx. MoI
    public static final double COEFFICIENT_OF_FRICTION = 1.5;

    public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    public static final RobotConfig CONFIG = new RobotConfig(
            DrivetrainConstants.ROBOT_MASS_KG,
            DrivetrainConstants.ROBOT_MOI_KGxMxM,
            new ModuleConfig(
                    CompbotTunerConstants.WHEEL_RADIUS.in(Meters),
                    CompbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond),
                    DrivetrainConstants.COEFFICIENT_OF_FRICTION,
                    DCMotor.getKrakenX60(1).withReduction(CompbotTunerConstants.DRIVE_GEAR_RATIO),
                    60,
                    1),
            new Translation2d(CompbotTunerConstants.FRONT_LEFT_XPOS, CompbotTunerConstants.FRONT_LEFT_YPOS),
            new Translation2d(CompbotTunerConstants.FRONT_RIGHT_XPOS, CompbotTunerConstants.FRONT_RIGHT_YPOS),
            new Translation2d(CompbotTunerConstants.BACK_LEFT_XPOS, CompbotTunerConstants.BACK_LEFT_YPOS),
            new Translation2d(CompbotTunerConstants.BACK_RIGHT_XPOS, CompbotTunerConstants.BACK_RIGHT_YPOS));
}
