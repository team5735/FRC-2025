package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkFlex sparkFlex_pull = new SparkFlex(Constants.MANIPULATOR_MOTOR_BOTTOM_ID, MotorType.kBrushless);
    private final SparkFlexConfig pull_config = new SparkFlexConfig();
    private final SparkFlex sparkFlex_push = new SparkFlex(Constants.MANIPULATOR_MOTOR_TOP_ID, MotorType.kBrushless);
    private final SparkFlexConfig push_config = new SparkFlexConfig();
    private final DigitalInput m_switch = new DigitalInput(Constants.INTAKE_BEAM_ID);


    public CoralSubsystem() {
        sparkFlex_pull.configure(pull_config.inverted(true), null, null);
        sparkFlex_push.configure(push_config.inverted(true), null, null);
    }

    public void top() {
        sparkFlex_pull.setVoltage(CoralConstants.TOP_VOLTS);
    }

    public void bottom() {
        sparkFlex_push.setVoltage(CoralConstants.BOTTOM_VOLTS);
    }

    public void stop() {
        sparkFlex_push.setVoltage(0);
        sparkFlex_pull.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intakeSwitchStatus", getSwitchStatus());
    }

    public boolean getSwitchStatus() {
        return m_switch.get();
    }

    public Command topStopCommand() {
        return startEnd(() -> top(), () -> stop());
    }

    public Command bottomStopCommand() {
        return startEnd(() -> bottom(), () -> stop());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }

    public Trigger beamBreakEngaged() {
        return new Trigger(() -> !getSwitchStatus());
    }
}
