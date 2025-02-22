package frc.robot.constants.drivetrain;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public interface DrivetrainConstants {
    public PIDConstants getAutoPosConstants();

    public PIDConstants getAutoRotConstants();

    public Distance getRobotTotalWidth();

    public Distance getRobotTotalLength();

    public Distance getPigeonToRobotFront();

    public PathConstraints getPathFollowConstraints();

    public double getSpinKp();

    public double getSpinKi();

    public double getSpinKd();

    public double getSpinKs();

    public double getSpinKv();

    public double getSpinKa();

    public LinearVelocity getDefaultSpeed();

    public AngularVelocity getDefaultRotationalRate();

    public Mass getRobotMass();

    public Distance getMaxWheelDistance();

    public double getRobotMoiKgxMxM();

    public double getCoefficientOfFriction();

    public RobotConfig getConfig();

    public Translation2d getPigeonToCenterOfRotation();
}
