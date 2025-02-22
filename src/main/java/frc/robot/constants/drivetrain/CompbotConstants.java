package frc.robot.constants.drivetrain;

import static edu.wpi.first.units.Units.*;

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

public class CompbotConstants implements DrivetrainConstants {
    @Override
    public PIDConstants getAutoPosConstants() {
        return new PIDConstants(10, 0);
    }

    @Override
    public PIDConstants getAutoRotConstants() {
        return new PIDConstants(15, 0);
    }

    @Override
    public Distance getRobotTotalWidth() {
        throw new Todo();
    }

    @Override
    public Distance getRobotTotalLength() {
        throw new Todo();
    }

    @Override
    public Distance getPigeonToRobotFront() {
        throw new Todo();
    }

    @Override
    public PathConstraints getPathFollowConstraints() {
        throw new Todo();
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
        return 0.08504;
    }

    @Override
    public double getSpinKv() {
        return 0.013474;
    }

    @Override
    public double getSpinKa() {
        return 0.0050825;
    }

    @Override
    public LinearVelocity getDefaultSpeed() {
        return MetersPerSecond.of(4);
    }

    @Override
    public AngularVelocity getDefaultRotationalRate() {
        return RotationsPerSecond.of(0.25);
    }

    @Override
    public Mass getRobotMass() {
        return Kilograms.of(25.35);
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

    public Translation2d getPigeonToCenterOfRotation(){
        return new Translation2d(Inches.of(0), Inches.of(0));
    }
}
