package frc.robot.constants.drivetrain;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

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
        throw new Todo();
    }

    @Override
    public double getCoefficientOfFriction() {
        throw new Todo();
    }

    @Override
    public RobotConfig getConfig() {
        throw new Todo();
    }

}
