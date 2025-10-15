package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.Height;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ElevatorSubsystem extends SubsystemBase {
    private SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withMechanismCircumference(Inches.of(ElevatorConstants.INCHES_PER_ENCODER_COUNTS))
            .withClosedLoopController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD,
                    ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION)
            .withTelemetry("elevator_motor", TelemetryVerbosity.HIGH)
            .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(1)))
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST);

    private final TalonFX krakenRight = new TalonFX(Constants.ELEVATOR_KRAKEN_RIGHT_ID);
    private final TalonFX krakenLeft = new TalonFX(Constants.ELEVATOR_KRAKEN_LEFT_ID);

    private SmartMotorController rightMotorController = new TalonFXWrapper(krakenRight, DCMotor.getKrakenX60(1),
            config);

    private ElevatorConfig elevatorConfig = new ElevatorConfig(rightMotorController)
            .withStartingHeight(ElevatorConstants.BASE_HEIGHT)
            .withHardLimits(ElevatorConstants.BASE_HEIGHT, ElevatorConstants.MAX_HEIGHT)
            .withTelemetry("elevator", TelemetryVerbosity.HIGH);

    private Elevator elevator = new Elevator(elevatorConfig);

    private Height currentHeight;

    public ElevatorSubsystem() {
        krakenLeft.setControl(new Follower(Constants.ELEVATOR_KRAKEN_RIGHT_ID, true));
    }

    @Override
    public void periodic() {
        elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        elevator.simIterate();
    }

    public Trigger atLevel(Height target) {
        return elevator.isNear(target.height, ElevatorConstants.AT_LEVEL_THRESHOLD);
    }

    /**
     * Returns a command to go to the specified level.
     *
     * This function returns a {@link Command} that, when run, tells this elevator
     * to go to the specified level. The command finishes when {@link #atLevel}'s
     * {@link Trigger} is true.
     *
     * @param level the {@link Height} to set the elevator to
     * @return a {@link Command} that sets the elevator to the specified height
     *
     * @example
     *          ```
     *          driveController.a().onTrue(elevator.getSetLevel(Height.BASE));
     *          ```
     */
    public Command getSetLevel(Height level) {
        return runOnce(() -> currentHeight = level)
                .alongWith(elevator.setHeight(level.height)
                        .until(atLevel(level)))
                .withName("set level to " + level);
    }

    /**
     * Returns a command to score at the specified level.
     *
     * This function returns a {@link Command} that, when run, tells the elevator to
     * go to the specified level and then score. This command finishes when both
     * actions are complete. This command chains {@link #getSetLevel(Height)} and
     * the passed {@link CoralSubsystem}'s
     * {@link CoralSubsystem#outputBasedOnLevel(Supplier)}
     * command.
     *
     * @param level   the target level
     * @param coraler the coral subsystem to use for scoring
     * @return a command to go to the target level and score a coral
     *
     * @example
     *          ```
     *          driveController.b().onTrue(elevator.getSetLevelAndCoral(Height.L4,
     *          coraler));
     *          ```
     */
    public Command getSetLevelAndCoral(Height level, CoralSubsystem coraler) {
        return getSetLevel(level)
                .andThen(coraler.outputBasedOnLevel(() -> currentHeight))
                .withName("set level to " + level + " and coral");
    }

    /**
     * Returns a Command that sets this elevator to a specific speed in the range
     * [-1, 1].
     *
     * @param elevatorSpeed the speed between -1 and 1 to set
     * @return a Command to set to that speed
     */
    public Command getSetSpeed(double elevatorSpeed) {
        return elevator.set(elevatorSpeed).withName("set elevator to " + elevatorSpeed);
    }

    /**
     * Returns a command to run a SysID routine on this elevator.
     *
     * This function returns a System Identification routine used for determining
     * the PID coefficients. This makes use of
     * {@link Elevator#sysId(Voltage, Velocity, Time)} as opposed to
     * {@link SysIdRoutine}.
     *
     * @return a SysID routine
     *
     * @example
     *          ```
     *          driveController.menu().onTrue(elevator.sysId());
     *          ```
     */
    public Command sysId() {
        return elevator.sysId(Volts.of(0.5), Volts.of(0.1).per(Second), Seconds.of(30)).withName("elevator sysID");
    }
}
