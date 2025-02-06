package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
    private final SparkMax sparkMax_feed = new SparkMax(Constants.ALGAE_FALCON_ID, MotorType.kBrushless);
    private final SparkMaxConfig pull_config = new SparkMaxConfig();

    public AlgaeSubsystem() {
        sparkMax_feed.configure(pull_config.inverted(true), null, null);
    }

    public void feed() {
        sparkMax_feed.setVoltage(AlgaeConstants.ALGAE_FALCON_VOLTS);
    }

    public void spit() {
        sparkMax_feed.setVoltage(-AlgaeConstants.ALGAE_FALCON_VOLTS);
    }

    public void stop() {
        sparkMax_feed.setVoltage(0);
    }

    public Command pullStopCommand() {
        return startEnd(() -> feed(), () -> stop());
    }

    public Command pushStopCommand() {
        return startEnd(() -> spit(), () -> stop());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }
}
