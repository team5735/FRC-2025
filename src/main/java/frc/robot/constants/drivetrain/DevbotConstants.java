package frc.robot.constants.drivetrain;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public class DevbotConstants implements DrivetrainConstants {
    @Override
    public PIDConstants getAutoPosConstants() {
        return new PIDConstants(15, 0); // TODO
    }

    @Override
    public PIDConstants getAutoRotConstants() {
        return new PIDConstants(10, 0); // TODO
    }

    @Override
    public double getRotKs() {
        return 0; // TODO
    }

    @Override
    public double getRotKv() {
        return 0; // TODO
    }

    @Override
    public double getRotKa() {
        return 0; // TODO
    }

    @Override
    public Distance getRobotTotalWidth() {
        return Inches.of(30).plus(getBumperWidth().times(2));
    }

    @Override
    public Distance getRobotTotalLength() {
        return Inches.of(30).plus(getBumperWidth().times(2));
    }

    @Override
    public Distance getPigeonToRobotFront() {
        return getRobotTotalLength().div(2);
    }

    @Override
    public PathConstraints getPathFollowConstraints() {
        return new PathConstraints(
                MetersPerSecond.of(4),
                MetersPerSecondPerSecond.of(2),
                DegreesPerSecond.of(540),
                DegreesPerSecondPerSecond.of(270));
    }

    @Override
    public LinearVelocity getDefaultSpeed() {
        return MetersPerSecond.of(2);
    }

    @Override
    public LinearVelocity getSlowSpeed() {
        return MetersPerSecond.of(1);
    }

    @Override
    public AngularVelocity getDefaultRotationalRate() {
        return RotationsPerSecond.of(0.25);
    }

    @Override
    public AngularVelocity getSlowRotationalRate() {
        return RotationsPerSecond.of(0.0625);
    }

    @Override
    public Mass getRobotMass() {
        return Kilograms.of(24.75);
    }

    @Override
    public Distance getDrivetrainWidth() {
        return Inches.of(30);
    }

    @Override
    public double getRobotMoiKgxMxM() {
        return getRobotMass().in(Kilograms) * getDrivetrainWidth().in(Meters) / 2 * getRotKa()
                / DevbotTunerConstants.DEFAULT_DRIVE_CONSTANTS.kA;
    }

    @Override
    public double getCoefficientOfFriction() {
        return 1.5;
    }

    private final RobotConfig config = new RobotConfig(
            getRobotMass().in(Kilograms),
            getRobotMoiKgxMxM(),
            new ModuleConfig(
                    DevbotTunerConstants.WHEEL_RADIUS.in(Meters),
                    DevbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond),
                    getCoefficientOfFriction(),
                    DCMotor.getKrakenX60(1).withReduction(DevbotTunerConstants.DRIVE_GEAR_RATIO),
                    60,
                    1),
            new Translation2d(DevbotTunerConstants.FL_XPOS, DevbotTunerConstants.FL_YPOS),
            new Translation2d(DevbotTunerConstants.FR_XPOS, DevbotTunerConstants.FR_YPOS),
            new Translation2d(DevbotTunerConstants.BL_XPOS, DevbotTunerConstants.BL_YPOS),
            new Translation2d(DevbotTunerConstants.BR_XPOS, DevbotTunerConstants.BR_YPOS));

    @Override
    public RobotConfig getConfig() {
        return config;
    }

    public Translation2d getPigeonToCenterOfRotation() {
        return new Translation2d(Inches.of(0), Inches.of(0));
    }

    public Distance getBumperWidth() {
        return Inches.of(3.5);
    }
}
