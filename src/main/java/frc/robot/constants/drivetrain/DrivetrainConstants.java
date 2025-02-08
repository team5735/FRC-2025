package frc.robot.constants.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    public static final double ROTATION_KP = 0.014787;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0;
    public static final double ROTATION_KS = 0.08504;
    public static final double ROTATION_KV = 0.013474;
    public static final double ROTATION_KA = 0.0050825;

    public static final double DEADBAND = 0.15;
    public static final double SPEED_MPS = 4;
    public static final double SPIN_RPS = 0.25;
    public static final double ROBOT_MASS_KG = 25.35;
    public static final double MAX_WHEEL_DISTANCE_M = Units.inchesToMeters(25);
    public static final double ROBOT_MOI_KGxMxM = 
            ROBOT_MASS_KG * MAX_WHEEL_DISTANCE_M / 2 * ROTATION_KA / CompbotTunerConstants.driveGains.kA; // Formula for approx. MoI
    public static final double COEFFICIENT_OF_FRICTION = 1.5;

    public static final RobotConfig CONFIG = new RobotConfig(
        DrivetrainConstants.ROBOT_MASS_KG,
        DrivetrainConstants.ROBOT_MOI_KGxMxM,
        new ModuleConfig(
            CompbotTunerConstants.kWheelRadius.in(Meters),
            CompbotTunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            DrivetrainConstants.COEFFICIENT_OF_FRICTION,
            DCMotor.getKrakenX60(1).withReduction(CompbotTunerConstants.kDriveGearRatio),
            60,
            1
        ),
        new Translation2d(CompbotTunerConstants.kFrontLeftXPos, CompbotTunerConstants.kFrontLeftYPos),
        new Translation2d(CompbotTunerConstants.kFrontRightXPos, CompbotTunerConstants.kFrontRightYPos),
        new Translation2d(CompbotTunerConstants.kBackLeftXPos, CompbotTunerConstants.kBackLeftYPos),
        new Translation2d(CompbotTunerConstants.kBackRightXPos, CompbotTunerConstants.kBackRightYPos)
    );
}
