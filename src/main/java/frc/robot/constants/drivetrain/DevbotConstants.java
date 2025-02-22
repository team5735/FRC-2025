package frc.robot.constants.drivetrain;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
        throw new Todo();
    }

    @Override
    public PIDConstants getAutoRotConstants() {
        throw new Todo();
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
        throw new Todo();
    }

    @Override
    public double getSpinKv() {
        throw new Todo();
    }

    @Override
    public double getSpinKa() {
        throw new Todo();
    }

    @Override
    public LinearVelocity getDefaultSpeed() {
        throw new Todo();
    }

    @Override
    public AngularVelocity getDefaultRotationalRate() {
        throw new Todo();
    }

    @Override
    public Mass getRobotMass() {
        throw new Todo();
    }

    @Override
    public Distance getMaxWheelDistance() {
        throw new Todo();
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

}
