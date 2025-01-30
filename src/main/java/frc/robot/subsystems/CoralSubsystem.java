package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax sparkMax_pull = new SparkMax(Constants.MANIPULATOR_MOTOR_BOTTOM_ID, MotorType.kBrushless);
    private final SparkMaxConfig pull_config = new SparkMaxConfig();

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

    public Command pullStopCommand() {
        return startEnd(() -> pull(), () -> stop());
    }

    public Command pushStopCommand() {
        return startEnd(() -> push(), () -> stop());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }

}
