package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static final DrivetrainType ROBOT_DRIVETRAIN = DrivetrainType.DEVBOT;

    public static final int ALGAE_FALCON_ID = 57;

    public static final int CORAL_MOTOR_TOP_ID = 5;
    public static final int CORAL_MOTOR_BOTTOM_ID = 4;

    public static final int INTAKE_BEAM_PIN = 0;

    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int SUBSYSTEM_CONTROLLER_PORT = 1;

    public static final double DEADBAND = 0.15;

    public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    // if you don't feel like picking for PIDs
    public static final double TOLERANCE = 0.005;

    public static final double PID_P = 6;
    public static final double PID_I = 2;
    public static final double PID_D = 0;

    public static final DrivetrainType DRIVETRAIN_TYPE = DrivetrainType.DEVBOT;

    public enum DrivetrainType {
        DEVBOT,
        COMPBOT
    }

    public static final int CANDLE_ID = 33;
}
