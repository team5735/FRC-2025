package frc.robot.constants.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.DrivetrainSubsystem;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class CompbotTunerConstants {
    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs DEFAULT_STEER_GAINS = new Slot0Configs()
            .withKP(29.519).withKI(0).withKD(1.7115)
            .withKS(0.052654).withKV(1.3981).withKA(0.092636)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKP(1.3577).withKI(0).withKD(0)
            .withKS(0.046541).withKV(1.7106).withKA(0.14237);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
    // RemoteCANcoder
    private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current SLIP_CURRENT = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these
    // cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API
    // documentation.
    private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS = new TalonFXConfiguration();
    private static final TalonFXConfiguration STEER_INITIAL_CONFIGS = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            // Swerve azimuth does not require much torque output, so we can set a
                            // relatively low
                            // stator current limit to help avoid brownouts without impacting performance.
                            .withStatorCurrentLimit(Amps.of(60))
                            .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus CANBus = new CANBus("Boat", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(12 / DRIVE_GAINS.kV);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    private static final double COUPLE_RATIO = 3.7;

    public static final double DRIVE_GEAR_RATIO = 4.725;
    private static final double STEER_GEAR_RATIO = 12.1;
    public static final Distance WHEEL_RADIUS = Inches.of(2);

    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    private static final int PIGEON_ID = 13;

    // These are only used for simulation
    private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
    private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(CANBus.getName())
            .withPigeon2Id(PIGEON_ID)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLE_RATIO)
            .withWheelRadius(WHEEL_RADIUS)
            .withSteerMotorGains(DEFAULT_STEER_GAINS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSlipCurrent(SLIP_CURRENT)
            .withSpeedAt12Volts(SPEED_AT_12_VOLTS)
            .withDriveMotorType(DRIVE_MOTOR_TYPE)
            .withSteerMotorType(STEER_MOTOR_TYPE)
            .withFeedbackSource(STEER_FEEDBACK_TYPE)
            .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
            .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(STEER_INERTIA)
            .withDriveInertia(DRIVE_INERTIA)
            .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
            .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

    // Front Left
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 8;
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 7;
    private static final int FRONT_LEFT_ENCODER_ID = 9;
    private static final Angle FRONT_LEFT_ENCODER_OFFSET = Rotations.of(-0.138427734375);
    private static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
    private static final boolean FRONT_LEFT_ENCODER_INVERTED = false;

    public static final Slot0Configs FL_STEER_GAINS = new Slot0Configs()
            .withKP(32.227).withKI(0).withKD(0.3)
            .withKS(0.077652).withKV(1.3923).withKA(0.18848)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final Distance FRONT_LEFT_XPOS = Inches.of(12.5);
    public static final Distance FRONT_LEFT_YPOS = Inches.of(12.5);

    // Front Right
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 10;
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 11;
    private static final int FRONT_RIGHT_ENCODER_ID = 12;
    private static final Angle FRONT_RIGHT_ENCODER_OFFSET = Rotations.of(0.475341796875);
    private static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
    private static final boolean FRONT_RIGHT_ENCODER_INVERTED = false;

    public static final Slot0Configs FR_STEER_GAINS = new Slot0Configs()
            .withKP(30.398).withKI(0).withKD(0.3)
            .withKS(0.041413).withKV(1.4116).withKA(0.10013)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final Distance FRONT_RIGHT_XPOS = Inches.of(12.5);
    public static final Distance FRONT_RIGHT_YPOS = Inches.of(-12.5);

    // Back Left
    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 4;
    private static final int BACK_LEFT_STEER_MOTOR_ID = 5;
    private static final int BACK_LEFT_ENCODER_ID = 6;
    private static final Angle BACK_LEFT_ENCODER_OFFSET = Rotations.of(0.18798828125);
    private static final boolean BACK_LEFT_STEER_MOTOR_INVERTED = true;
    private static final boolean BACK_LEFT_ENCODER_INVERTED = false;

    public static final Slot0Configs BL_STEER_GAINS = new Slot0Configs()
            .withKP(28.073).withKI(0).withKD(0.3)
            .withKS(0.12149).withKV(1.3951).withKA(0.083189)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final Distance BACK_LEFT_XPOS = Inches.of(-12.5);
    public static final Distance BACK_LEFT_YPOS = Inches.of(12.5);

    // Back Right
    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 2;
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 1;
    private static final int BACK_RIGHT_ENCODER_ID = 3;
    private static final Angle BACK_RIGHT_ENCODER_OFFSET = Rotations.of(-0.422607421875);
    private static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;
    private static final boolean BACK_RIGHT_ENCODER_INVERTED = false;

    public static final Slot0Configs BR_STEER_GAINS = new Slot0Configs()
            .withKP(29.344).withKI(0).withKD(0.3)
            .withKS(0.068374).withKV(1.4079).withKA(0.09265)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final Distance BACK_RIGHT_XPOS = Inches.of(-12.5);
    public static final Distance BACK_RIGHT_YPOS = Inches.of(-12.5);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT = ConstantCreator
            .createModuleConstants(
                    FRONT_LEFT_STEER_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_ENCODER_ID,
                    FRONT_LEFT_ENCODER_OFFSET, FRONT_LEFT_XPOS, FRONT_LEFT_YPOS, INVERT_LEFT_SIDE,
                    FRONT_LEFT_STEER_MOTOR_INVERTED, FRONT_LEFT_ENCODER_INVERTED)
            .withSteerMotorGains(FL_STEER_GAINS);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT = ConstantCreator
            .createModuleConstants(
                    FRONT_RIGHT_STEER_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_ENCODER_ID,
                    FRONT_RIGHT_ENCODER_OFFSET, FRONT_RIGHT_XPOS, FRONT_RIGHT_YPOS, INVERT_RIGHT_SIDE,
                    FRONT_RIGHT_STEER_MOTOR_INVERTED, FRONT_RIGHT_ENCODER_INVERTED)
            .withSteerMotorGains(FR_STEER_GAINS);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT = ConstantCreator
            .createModuleConstants(
                    BACK_LEFT_STEER_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_ENCODER_ID, BACK_LEFT_ENCODER_OFFSET,
                    BACK_LEFT_XPOS, BACK_LEFT_YPOS, INVERT_LEFT_SIDE, BACK_LEFT_STEER_MOTOR_INVERTED,
                    BACK_LEFT_ENCODER_INVERTED)
            .withSteerMotorGains(BL_STEER_GAINS);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT = ConstantCreator
            .createModuleConstants(
                    BACK_RIGHT_STEER_MOTOR_ID, BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_ENCODER_ID,
                    BACK_RIGHT_ENCODER_OFFSET, BACK_RIGHT_XPOS, BACK_RIGHT_YPOS, INVERT_RIGHT_SIDE,
                    BACK_RIGHT_STEER_MOTOR_INVERTED, BACK_RIGHT_ENCODER_INVERTED)
            .withSteerMotorGains(BR_STEER_GAINS);

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    public static DrivetrainSubsystem createDrivetrain() {
        return new DrivetrainSubsystem(
                DrivetrainConstants, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);
    }

    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected
     * device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct the devices themselves. If they need the devices, they can access
         * them through getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param modules             Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct the devices themselves. If they need the devices, they can access
         * them through getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct the devices themselves. If they need the devices, they can access
         * them through getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve
         *                                  drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz
         *                                  on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry
         *                                  calculation
         *                                  in the form [x, y, theta]ᵀ, with units in
         *                                  meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision
         *                                  calculation
         *                                  in the form [x, y, theta]ᵀ, with units in
         *                                  meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency,
                    odometryStandardDeviation, visionStandardDeviation, modules);
        }
    }
}
