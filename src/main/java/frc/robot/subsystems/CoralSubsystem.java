package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkFlex vortex_top = new SparkFlex(Constants.MANIPULATOR_MOTOR_BOTTOM_ID, MotorType.kBrushless);
    private final SparkFlexConfig top_config = new SparkFlexConfig();
    private final SparkFlex vortex_bottom = new SparkFlex(Constants.MANIPULATOR_MOTOR_TOP_ID, MotorType.kBrushless);
    private final SparkFlexConfig bottom_config = new SparkFlexConfig();
    private final DigitalInput m_switch = new DigitalInput(Constants.INTAKE_BEAM_ID);


    public CoralSubsystem() {
        vortex_top.configure(top_config.inverted(true).idleMode(IdleMode.kBrake), null, null);
        vortex_bottom.configure(bottom_config.inverted(true).idleMode(IdleMode.kBrake), null, null);
    }

    private void intakeTop() {
        vortex_top.setVoltage(CoralConstants.INTAKE_TOP_VOLTS);
    }

    private void intakeBottom() {
        vortex_bottom.setVoltage(CoralConstants.INTAKE_BOTTOM_VOLTS);
    }

    private void troughTop() {
        vortex_top.setVoltage(CoralConstants.TROUGH_TOP_VOLTS);
    }

    private void troughBottom() {
        vortex_bottom.setVoltage(CoralConstants.TROUGH_BOTTOM_VOLTS);
    }

    private void branchTop() {
        vortex_top.setVoltage(CoralConstants.BRANCH_TOP_VOLTS);
    }

    private void branchBottom() {
        vortex_bottom.setVoltage(CoralConstants.BRANCH_BOTTOM_VOLTS);
    }

    private void stop() {
        vortex_top.setVoltage(0);
        vortex_bottom.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intakeSwitchStatus", getSwitchStatus());
    }

    public boolean getSwitchStatus() {
        return m_switch.get();
    }

    public Trigger beamBreakEngaged() {
        return new Trigger(() -> !getSwitchStatus());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }

    public Command feedInCommand() {
        return runOnce(() -> intakeBottom())
            .withDeadline(new WaitCommand(CoralConstants.FEED_DELAY_SECONDS))
            .andThen(runOnce(() -> intakeTop()))
            .until(beamBreakEngaged())
            .andThen(runOnce(() -> stop()));
    }

    public Command troughCommand() {
        return runOnce(() -> troughTop())
            .alongWith(runOnce(() -> troughBottom())); //awaiting buttom mapping
    }

    public Command branchCommand() {
        return runOnce(() -> branchTop())
            .alongWith(runOnce(() -> branchBottom())); //awaiting buttom mapping
    }
}