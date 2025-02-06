package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Constants;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystems extends SubsystemBase {
    private final SparkMax sparkMax_feed = new SparkMax(Constants.FEEDER_NEO_ID, MotorType.kBrushless);
    private final SparkMaxConfig feed_config = new SparkMaxConfig();

    public void FeederSubsystem() {
        sparkMax_feed.configure(feed_config.inverted(true), null, null);
    }

    public void feed() {
        sparkMax_feed.setVoltage(FeederConstants.FEEDER_NEO_VOLTS);
    }

    public void unfeed() {
        sparkMax_feed.setVoltage(-FeederConstants.FEEDER_NEO_VOLTS);
    }

    public void stop() {
        sparkMax_feed.setVoltage(0);
    }

    public Command feedStopCommand() {
        return startEnd(() -> feed(), () -> stop());
    }

    public Command unfeedStopCommand() {
        return startEnd(() -> unfeed(), () -> stop());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }

}
