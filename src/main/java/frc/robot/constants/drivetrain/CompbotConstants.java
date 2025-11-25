package frc.robot.constants.drivetrain;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

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

public class CompbotConstants implements DrivetrainConstants {
    @Override
    public PIDConstants getAutoPosConstants() {
        return new PIDConstants(17.5, 0);
    }

    @Override
    public PIDConstants getAutoRotConstants() {
        return new PIDConstants(12, 0);
    }

    @Override
    public Distance getRobotTotalWidth() {
        return Inches.of(38);
    }

    @Override
    public Distance getRobotTotalLength() {
        return Inches.of(38);
    }

    @Override
    public Distance getPigeonToRobotFront() {
        return getRobotTotalLength().div(2);
    }

    @Override
    public PathConstraints getPathFollowConstraints() {
        return new PathConstraints(
                MetersPerSecond.of(1),
                MetersPerSecondPerSecond.of(0.5),
                DegreesPerSecond.of(60),
                DegreesPerSecondPerSecond.of(30)); // TODO add and test values
    }

    @Override
    public double getSpinKp() {
        return 0.014687;
    }

    @Override
    public double getSpinKi() {
        return 0;
    }

    @Override
    public double getSpinKd() {
        return 0;
    }

    @Override
    public double getSpinKs() {
        return 0.16327;
    }

    @Override
    public double getSpinKv() {
        return 0.80246;
    }

    @Override
    public double getSpinKa() {
        return 0.185;
    }

    @Override
    public LinearVelocity getDefaultSpeed() {
        return MetersPerSecond.of(2);
    }

    @Override
    public LinearVelocity getSlowSpeed() {
        return MetersPerSecond.of(0.5);
    }

    @Override
    public AngularVelocity getDefaultRotationalRate() {
        return DegreesPerSecond.of(180);
    }

    @Override
    public AngularVelocity getSlowRotationalRate() {
        return DegreesPerSecond.of(27);
    }

    @Override
    public Mass getRobotMass() {
        return Kilograms.of(59.05);
    }

    @Override
    public Distance getMaxWheelDistance() {
        return Inches.of(25);
    }

    @Override
    public double getRobotMoiKgxMxM() {
        return getRobotMass().in(Kilograms) * getMaxWheelDistance().in(Meters) / 2 * getSpinKa()
                / CompbotTunerConstants.DRIVE_GAINS.kA;
    }

    @Override
    public double getCoefficientOfFriction() {
        return 1.5;
    }

    private final RobotConfig config = new RobotConfig(
            getRobotMass().in(Kilograms),
            getRobotMoiKgxMxM(),
            new ModuleConfig(
                    CompbotTunerConstants.WHEEL_RADIUS.in(Meters),
                    CompbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond),
                    getCoefficientOfFriction(),
                    DCMotor.getKrakenX60(1).withReduction(CompbotTunerConstants.DRIVE_GEAR_RATIO),
                    60,
                    1),
            new Translation2d(CompbotTunerConstants.FRONT_LEFT_XPOS, CompbotTunerConstants.FRONT_LEFT_YPOS),
            new Translation2d(CompbotTunerConstants.FRONT_RIGHT_XPOS, CompbotTunerConstants.FRONT_RIGHT_YPOS),
            new Translation2d(CompbotTunerConstants.BACK_LEFT_XPOS, CompbotTunerConstants.BACK_LEFT_YPOS),
            new Translation2d(CompbotTunerConstants.BACK_RIGHT_XPOS, CompbotTunerConstants.BACK_RIGHT_YPOS));

    @Override
    public RobotConfig getConfig() {
        return config;
    }

    public Translation2d getPigeonToCenterOfRotation() {
        return new Translation2d(Inches.of(-0.75), Inches.of(0));
    }

    public Distance getBumperWidth() {
        return Inches.of(3.5); // TODO
    }
}
