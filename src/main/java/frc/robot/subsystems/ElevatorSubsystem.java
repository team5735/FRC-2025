package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX krakenRight = new TalonFX(Constants.ELEVATOR_KRAKEN_RIGHT_ID);
    private final TalonFX krakenLeft = new TalonFX(Constants.ELEVATOR_KRAKEN_LEFT_ID);
    private ProfiledPIDController pid = new ProfiledPIDController(
            ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD,
            new Constraints(
                    ElevatorConstants.MAX_VELOCITY.in(MetersPerSecond),
                    ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond)));
    private ElevatorFeedforward ff = new ElevatorFeedforward(
            ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);
    private ElevatorConstants.Level activeLevel = ElevatorConstants.Level.BASE;

    private final SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(5), null, null),
            new SysIdRoutine.Mechanism(
                    v -> krakenRight.setVoltage(v.in(Volts)),
                    null,
                    this));

    public ElevatorSubsystem() {
        krakenRight.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        krakenLeft.setControl(new Follower(Constants.ELEVATOR_KRAKEN_RIGHT_ID, true));
        resetMeasurement();

        SmartDashboard.putNumber("ElevatorPosMeters", ElevatorConstants.BASE_HEIGHT.in(Meters));
        SmartDashboard.putNumber("ElevatorVelocityMPS", 0);
        SmartDashboard.putNumber("ElevatorVolts", krakenRight.getMotorVoltage().getValue().in(Volts));
        this.setDefaultCommand(toLevelCommand(ElevatorConstants.Level.BASE));
    }

    public boolean isAtRest() {
        return (pid.getGoal() == ElevatorConstants.Level.BASE.stateSupplier.get()) && pid.atGoal();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ElevatorPosMeters", getPosition().in(Meters));
        SmartDashboard.putNumber(
                "ElevatorVelocityMPS",
                FeetPerSecond.of(
                        krakenRight.getVelocity().getValue().in(RotationsPerSecond)
                                * ElevatorConstants.ROTATIONS_TO_FEET)
                        .in(MetersPerSecond));
        SmartDashboard.putNumber("ElevatorVolts", krakenRight.getMotorVoltage().getValue().in(Volts));
    }

    private void setPIDVolts() {
        pid.setGoal(activeLevel.stateSupplier.get());

        double voltsToSet = pid.calculate(getPosition().in(Meters)) + ff.calculate(pid.getSetpoint().velocity);
        if (isAtRest())
            voltsToSet = 0; // TODO check if better logic is needed here

        krakenRight.setVoltage(voltsToSet);
    }

    public Distance getPosition() {
        return Feet.of(krakenRight.getPosition().getValue().in(Rotations) * ElevatorConstants.ROTATIONS_TO_FEET);
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

    private void resetMeasurement() {
        resetMeasurement(Feet.of(0));
    }

    private void resetMeasurement(Distance height) {
        krakenRight.setPosition(Rotations.of(height.in(Feet) / ElevatorConstants.ROTATIONS_TO_FEET));
    }
}
