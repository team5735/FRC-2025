package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.Level;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX krakenRight = new TalonFX(Constants.ELEVATOR_KRAKEN_RIGHT_ID);
    private final TalonFX krakenLeft = new TalonFX(Constants.ELEVATOR_KRAKEN_LEFT_ID);

    private final SparkMax dummyMax = new SparkMax(Constants.ELEVATOR_ENCODER_ID, MotorType.kBrushed);

    private boolean enabled = true;

    private ProfiledPIDController pid = new ProfiledPIDController(
            ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD,
            new Constraints(
                    ElevatorConstants.MAX_VELOCITY.in(MetersPerSecond),
                    ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond)));
    private ElevatorFeedforward ff = new ElevatorFeedforward(
            ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);

    private ElevatorConstants.Level activeLevel = ElevatorConstants.Level.BASE;

    private final SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.1).per(Second), Volts.of(0.5), Seconds.of(30), null),
            new SysIdRoutine.Mechanism(
                    v -> krakenRight.setVoltage(v.in(Volts)),
                    null,
                    this));

    public ElevatorSubsystem() {
        krakenRight.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        krakenLeft.setControl(new Follower(Constants.ELEVATOR_KRAKEN_RIGHT_ID, true));

        dummyMax.configure(new SparkMaxConfig().apply(new EncoderConfig().inverted(true).positionConversionFactor(1)),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetMeasurement();

        SmartDashboard.putNumber("Elevator/PosMeters", ElevatorConstants.BASE_HEIGHT.in(Meters));
        SmartDashboard.putNumber("Elevator/VelocityMPS", 0);
        SmartDashboard.putNumber("Elevator/Volts",
                krakenRight.getMotorVoltage().getValue().in(Volts));

        SmartDashboard.putNumber("Elevator/TuningVolts", 0);

        SmartDashboard.putString("Elevator/ActiveLevel", activeLevel.name());
        SmartDashboard.putString("Elevator/ActiveLevelColor", activeLevel.levelColor.toHexString());
        SmartDashboard.putNumber("Elevator/HeightTargetFeet", ElevatorConstants.BASE_HEIGHT.in(Units.Feet));

        pid.setTolerance(0.02); // 2cm

        this.setDefaultCommand(toLevelCommand(ElevatorConstants.Level.BASE));
    }

    public boolean isAtRest() {
        return (activeLevel == Level.BASE) && pid.atGoal();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/PosMeters", getPosition().in(Meters));
        SmartDashboard.putNumber("Elevator/PosInches", getPosition().in(Inches));
        SmartDashboard.putNumber("Elevator/EncoderRots", krakenRight.getPosition().getValue().in(Rotations));
        SmartDashboard.putNumber("Elevator/NewEncoderRots", dummyMax.getEncoder().getPosition());
        SmartDashboard.putNumber(
                "Elevator/VelocityMPS",
                InchesPerSecond.of(
                        krakenRight.getVelocity().getValue().in(RotationsPerSecond)
                                * ElevatorConstants.INCHES_PER_ENCODER_COUNTS)
                        .in(MetersPerSecond));
        SmartDashboard.putNumber("Elevator/Volts",
                krakenRight.getMotorVoltage().getValue().in(Volts) + 0.00001 * Math.random()); // I am hacker genius
        SmartDashboard.putString("Elevator/ActiveLevel", activeLevel.name());
        SmartDashboard.putString("Elevator/ActiveLevelColor", activeLevel.levelColor.toHexString());
        SmartDashboard.putNumber("Elevator/PosSetpointMet", pid.getSetpoint().position);
        SmartDashboard.putNumber("Elevator/VelSetpointMPS", pid.getSetpoint().velocity);
        SmartDashboard.putBoolean("Elevator/IsEnable", enabled);
        if (getPosition().in(Meters) < 0) {
            resetMeasurement();
        }

        // if (getPosition().in(Meters) > ElevatorConstants.MAX_HEIGHT.in(Meters)) {
        // resetMeasurement(ElevatorConstants.MAX_HEIGHT);
        // }
    }

    private void setPIDVolts() {
        pid.setGoal(activeLevel.stateSupplier.get());

        double voltsToSet = pid.calculate(getPosition().in(Meters)) +
                ff.calculate(pid.getSetpoint().velocity);
        if (isAtRest())
            voltsToSet = 0; // TODO check if better logic is needed here

        if (enabled) {
            krakenRight.setVoltage(voltsToSet);
        }
    }

    public Distance getPosition() {
        return Inches
                .of(krakenRight.getPosition().getValue().in(Rotations) * ElevatorConstants.INCHES_PER_ENCODER_COUNTS);
    }

    public Command toLevelCommand(ElevatorConstants.Level level) {
        return startRun(() -> setLevel(level), () -> setPIDVolts());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    private void setLevel(ElevatorConstants.Level level) {
        activeLevel = level;
    }

    public void resetMeasurement() {
        resetMeasurement(ElevatorConstants.BASE_HEIGHT);
        dummyMax.getEncoder().setPosition(0);
    }

    public void resetMeasurement(Distance height) {
        krakenRight.setPosition(Rotations.of(height.in(Inches) / ElevatorConstants.INCHES_PER_ENCODER_COUNTS));
    }

    public void swapEnableStatus() {
        enabled = !enabled;
    }

    public Command manualElevatorUp() {
        return startEnd(() -> {
            krakenRight.setVoltage(1.75);
            enabled = false;
        }, () -> krakenRight.setVoltage(ElevatorConstants.KG));
    }

    public Command manualElevatorDown() {
        return startEnd(() -> {
            krakenRight.setVoltage(-2.5 + ElevatorConstants.KG);
            enabled = false;
        }, () -> krakenRight.setVoltage(ElevatorConstants.KG));
    }

    public Level getActiveLevel() {
        return activeLevel;

    }

    public Command toLevelAndCoral(Level level, CoralSubsystem coral) {
        return toLevelCommand(level).until(() -> pid.atGoal())
                .andThen(toLevelCommand(level)).alongWith(coral.outputBasedOnLevel(this));
    }
}