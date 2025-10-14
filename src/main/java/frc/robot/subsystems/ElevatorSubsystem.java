package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    public Command getSetLevel(Height level) {
        return runOnce(() -> currentHeight = level).andThen(elevator.setHeight(level.height))
                .withName("set level to " + level);
    }

    public Command getSetLevelAndCoral(Height level, CoralSubsystem coraler) {
        return getSetLevel(level).until(atLevel(level))
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

    public Command sysId() {
        return elevator.sysId(Volts.of(0.5), Volts.of(0.1).per(Second), Seconds.of(30)).withName("elevator sysID");
    }
}
