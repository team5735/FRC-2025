package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax sparkMax_pull = new SparkMax(Constants.MANIPULATOR_MOTOR_BOTTOM_ID, MotorType.kBrushless);
    private final SparkMaxConfig pull_config = new SparkMaxConfig();
    private final DigitalInput m_switch = new DigitalInput(Constants.INTAKE_BEAM_ID);


    public CoralSubsystem() {
        sparkMax_pull.configure(pull_config.inverted(true), null, null);
    }

    public void pull() {
        sparkMax_pull.setVoltage(CoralConstants.TOP_VOLTS);
    }

    public void push() {
        sparkMax_pull.setVoltage(CoralConstants.BOTTOM_VOLTS);
    }

    public void stop() {
        sparkMax_pull.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intakeSwitchStatus", getSwitchStatus());
    }

    public boolean getSwitchStatus() {
        return m_switch.get();
    }

    public Command pullStopCommand() {
        return startEnd(() -> pull(), () -> stop());
    }

    public Command pushStopCommand() {
        return startEnd(() -> push(), () -> stop());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }


    public Trigger beamBreakEngaged() {
        return new Trigger(() -> !getSwitchStatus());
    }
}
