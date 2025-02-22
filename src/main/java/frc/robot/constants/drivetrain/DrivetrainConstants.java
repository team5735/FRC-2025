package frc.robot.constants.drivetrain;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public interface DrivetrainConstants {
    public static final PIDConstants AUTO_POS_CONSTANTS = new PIDConstants(10, 0);
    public static final PIDConstants AUTO_ROT_CONSTANTS = new PIDConstants(15, 0);

    public static final Distance ROBOT_TOTAL_WIDTH = Feet.of(0); // Left to Right
    public static final Distance ROBOT_TOTAL_LENGTH = Feet.of(0); // Front to Back
    // from 'robot center' (defined as the X formed by the swerve medules)
    public static final Distance PIGEON_TO_ROBOT_FRONT = Feet.of(0); // TODO: determine

    public static final PathConstraints PATH_FOLLOW_CONSTRAINTS = new PathConstraints(
            MetersPerSecond.of(0),
            MetersPerSecondPerSecond.of(0),
            DegreesPerSecond.of(0),
            DegreesPerSecondPerSecond.of(0)); // TODO fill values based on auto configs

    public static final double SPIN_KP = 0.014787;
    public static final double SPIN_KI = 0;
    public static final double SPIN_KD = 0;
    public static final double SPIN_KS = 0.08504;
    public static final double SPIN_KV = 0.013474;
    public static final double SPIN_KA = 0.0050825;

    public static final LinearVelocity DEFAULT_SPEED = MetersPerSecond.of(4);
    public static final AngularVelocity DEFAULT_ROTATIONAL_RATE = RotationsPerSecond.of(0.25);
    public static final Mass ROBOT_MASS = Kilograms.of(25.35);
    public static final Distance MAX_WHEEL_DISTANCE = Meters.of(Units.inchesToMeters(25));
    public static final double ROBOT_MOI_KGxMxM = ROBOT_MASS.in(Kilograms) * MAX_WHEEL_DISTANCE.in(Meters) / 2 * SPIN_KA
            / CompbotTunerConstants.DRIVE_GAINS.kA; // Formula for approx. MoI
    public static final double COEFFICIENT_OF_FRICTION = 1.5;
    public static final RobotConfig CONFIG = new RobotConfig(
            DrivetrainConstants.ROBOT_MASS.in(Kilograms),
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
