package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

public class TunablePIDController {
    /** PID controller. */
    protected PIDController controller;

    protected TunableNumber p, i, d;

    private NTDoubleSection doubles;

    /**
     * Creates a new PIDCommand, which controls the given output with a
     * PIDController.
     *
     * @param controller        the controller that controls the output.
     * @param measurementSource the measurement of the process variable
     * @param setpointSource    the controller's setpoint
     * @param useOutput         the controller's output
     * @param requirements      the subsystems required by this command
     */
    public TunablePIDController(
            String name,
            Subsystem... requirements) {
        this(name, Constants.PID_P, Constants.PID_I, Constants.PID_D, requirements);
    }

    public TunablePIDController(String name, double _p, double _i, double _d, Subsystem... requirements) {
        p = new TunableNumber("tunable_pid_commands", name + "_p", _p);
        i = new TunableNumber("tunable_pid_commands", name + "_i", _i);
        d = new TunableNumber("tunable_pid_commands", name + "_d", _d);

        doubles = new NTDoubleSection(name + " pid", "setpoint", "output", "measurement", "p", "i", "d");
    }

    public void setup(double setpoint) {
        setup(setpoint, Constants.TOLERANCE);
    }

    public void setup(double setpoint, double tolerance) {
        controller = new PIDController(p.get(), i.get(), d.get());
        doubles.set("p", p.get());
        doubles.set("i", i.get());
        doubles.set("d", d.get());

        controller.setSetpoint(setpoint);
        doubles.set("setpoint", setpoint);
        controller.setTolerance(tolerance);
    }

    public void reset() {
        controller.reset();
    }

    /**
     * Runs the PID calculation.
     * 
     * <p>
     * Outputs 0 if the controller is at the setpoint.
     * 
     * @param measurement
     * @return The controller output, or zero if atSetpoint.
     */
    public double calculate(double measurement) {
        double value = controller.calculate(measurement);
        doubles.set("measurement", measurement);
        doubles.set("output", value);
        return value;
    }

    /**
     * @return Whether the PIDController is within tolerance of the setpoint.
     */
    public boolean atSetpoint() {
        return this.controller.atSetpoint();
    }

    /**
     * Returns the PIDController used by the command.
     *
     * @return The PIDController
     */
    public PIDController getController() {
        return controller;
    }
}
