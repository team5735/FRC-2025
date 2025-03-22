package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable
            .getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
            .getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable
            .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
            .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency")
            .publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    private final Field2d sendableField = new Field2d();

    private final Sendable sendableState = new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                    "Front Left Angle",
                    () -> RobotContainer.drivetrain.getModule(0).getCurrentState().angle.getRadians(),
                    null);
            builder.addDoubleProperty(
                    "Front Left Velocity",
                    () -> RobotContainer.drivetrain.getModule(0).getCurrentState().speedMetersPerSecond,
                    null);

            builder.addDoubleProperty(
                    "Front Right Angle",
                    () -> RobotContainer.drivetrain.getModule(1).getCurrentState().angle.getRadians(),
                    null);
            builder.addDoubleProperty(
                    "Front Right Velocity",
                    () -> RobotContainer.drivetrain.getModule(1).getCurrentState().speedMetersPerSecond,
                    null);

            builder.addDoubleProperty(
                    "Back Left Angle",
                    () -> RobotContainer.drivetrain.getModule(2).getCurrentState().angle.getRadians(),
                    null);
            builder.addDoubleProperty(
                    "Back Left Velocity",
                    () -> RobotContainer.drivetrain.getModule(2).getCurrentState().speedMetersPerSecond,
                    null);

            builder.addDoubleProperty(
                    "Back Right Angle",
                    () -> RobotContainer.drivetrain.getModule(3).getCurrentState().angle.getRadians(),
                    null);
            builder.addDoubleProperty(
                    "Back Right Velocity",
                    () -> RobotContainer.drivetrain.getModule(3).getCurrentState().speedMetersPerSecond,
                    null);

            builder.addDoubleProperty(
                    "Robot Angle",
                    () -> RobotContainer.drivetrain.getPigeon2().getYaw().getValue().in(Units.Radians),
                    null);
        }
    };

    /** Accept the swerve drive state and telemeterize it to SmartDashboard. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        sendableField.setRobotPose(AutoBuilder.getCurrentPose());
        SmartDashboard.putData(sendableField);

        SmartDashboard.putData("Swerve State", sendableState);

        SmartDashboard.putNumberArray("DriveState/Pose", m_poseArray);
        SmartDashboard.putNumberArray("DriveState/ModuleStates", m_moduleStatesArray);
        SmartDashboard.putNumberArray("DriveState/ModuleTargets", m_moduleTargetsArray);
        SmartDashboard.putNumber("DriveState/OdometryPeriod/seconds", state.OdometryPeriod);

        SmartDashboard.putNumber("translation_position", state.Pose.getY());
        SmartDashboard.putNumber("translation_velocity", state.ModuleStates[0].speedMetersPerSecond);

        SmartDashboard.putNumber(
                "FL_steer_position",
                (RobotContainer.drivetrain.getModule(0).getCurrentState().angle.getRotations() + 1) % 1);
        SmartDashboard.putNumber(
                "FL_steer_velocity",
                RobotContainer.drivetrain.getModule(0).getSteerMotor().getVelocity().getValueAsDouble());

        SmartDashboard.putNumber(
                "FR_steer_position",
                (RobotContainer.drivetrain.getModule(1).getCurrentState().angle.getRotations() + 1) % 1);
        SmartDashboard.putNumber(
                "FR_steer_velocity",
                RobotContainer.drivetrain.getModule(1).getSteerMotor().getVelocity().getValueAsDouble());

        SmartDashboard.putNumber(
                "BL_steer_position",
                (RobotContainer.drivetrain.getModule(2).getCurrentState().angle.getRotations() + 1) % 1);
        SmartDashboard.putNumber(
                "BL_steer_velocity",
                RobotContainer.drivetrain.getModule(2).getSteerMotor().getVelocity().getValueAsDouble());

        SmartDashboard.putNumber(
                "BR_steer_position",
                (RobotContainer.drivetrain.getModule(3).getCurrentState().angle.getRotations() + 1) % 1);
        SmartDashboard.putNumber(
                "BR_steer_velocity",
                RobotContainer.drivetrain.getModule(3).getSteerMotor().getVelocity().getValueAsDouble());

        SmartDashboard.putNumber(
                "rotation_position",
                MathUtil.inputModulus(
                        RobotContainer.drivetrain.getPigeon2().getYaw().getValue().in(Units.Radians),
                        -Math.PI, Math.PI));
        SmartDashboard.putNumber(
                "rotation_velocity",
                RobotContainer.drivetrain.getPigeon2().getAngularVelocityZWorld().getValue()
                        .in(Units.RadiansPerSecond));

        SmartDashboard.putNumber(
                "FL_steer_setpoint",
                (RobotContainer.drivetrain.getModule(0).getTargetState().angle.getRotations() + 1) % 1);

        SmartDashboard.putNumber(
                "FL_drive_volts",
                RobotContainer.drivetrain.getModule(0).getDriveMotor().getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(
                "FL_steer_volts",
                RobotContainer.drivetrain.getModule(0).getSteerMotor().getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber(
                "FR_drive_volts",
                RobotContainer.drivetrain.getModule(1).getDriveMotor().getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(
                "FR_steer_volts",
                RobotContainer.drivetrain.getModule(1).getSteerMotor().getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber(
                "BL_drive_volts",
                RobotContainer.drivetrain.getModule(2).getDriveMotor().getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(
                "BL_steer_volts",
                RobotContainer.drivetrain.getModule(2).getSteerMotor().getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber(
                "BR_drive_volts",
                RobotContainer.drivetrain.getModule(3).getDriveMotor().getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(
                "BR_steer_volts",
                RobotContainer.drivetrain.getModule(3).getSteerMotor().getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber(
                "FL_drive_motor_rotations",
                RobotContainer.drivetrain.getModule(0).getDriveMotor().getRotorPosition().getValue()
                        .in(Units.Rotations));

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);

        /* Telemeterize the module states to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }
}
