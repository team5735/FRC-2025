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
import frc.robot.util.Todo;

public class DevbotConstants implements DrivetrainConstants {
    @Override
    public PIDConstants getAutoPosConstants() {
        return new PIDConstants(15, 0);
    }

    @Override
    public PIDConstants getAutoRotConstants() {
        return new PIDConstants(10, 0);
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
        return Inches.of(16).plus(getBumperWidth());
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
    public double getSpinKp() {
        throw new Todo();
    }

    @Override
    public double getSpinKi() {
        throw new Todo();
    }

    @Override
    public double getSpinKd() {
        throw new Todo();
    }

    @Override
    public double getSpinKs() {
        return 0.08652;
    }

    @Override
    public double getSpinKv() {
        return 1.0453;
    }

    @Override
    public double getSpinKa() {
        return 0.042742;
    }

    @Override
    public LinearVelocity getDefaultSpeed() {
        return MetersPerSecond.of(4); // TODO
    }

    @Override
    public AngularVelocity getDefaultRotationalRate() {
        return RotationsPerSecond.of(0.25); // TODO
    }

    @Override
    public Mass getRobotMass() {
        return Kilograms.of(37.50);
    }

    @Override
    public Distance getMaxWheelDistance() {
        return Inches.of(25);
    }

    @Override
    public double getRobotMoiKgxMxM() {
        return getRobotMass().in(Kilograms) * getMaxWheelDistance().in(Meters) / 2 * getSpinKa()
                / DevbotTunerConstants.DRIVE_GAINS.kA;
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
            new Translation2d(DevbotTunerConstants.FRONT_LEFT_XPOS, DevbotTunerConstants.FRONT_LEFT_YPOS),
            new Translation2d(DevbotTunerConstants.FRONT_RIGHT_XPOS, DevbotTunerConstants.FRONT_RIGHT_YPOS),
            new Translation2d(DevbotTunerConstants.BACK_LEFT_XPOS, DevbotTunerConstants.BACK_LEFT_YPOS),
            new Translation2d(DevbotTunerConstants.BACK_RIGHT_XPOS, DevbotTunerConstants.BACK_RIGHT_YPOS));

    @Override
    public RobotConfig getConfig() {
        return config;
    }

    public Translation2d getPigeonToCenterOfRotation() {
        return new Translation2d(Inches.of(2.5), Inches.of(0));
    }

    @Override
    public double getSlowMultiplier() {
        return 0.25;
    }

    public Distance getBumperWidth() {
        return Inches.of(3.5);
    }
}
