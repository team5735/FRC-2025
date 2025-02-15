package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Constants;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystems extends SubsystemBase {
    private final TalonFX falcon = new TalonFX(Constants.FEEDER_FALCON_ID);

    public void FeederSubsystem() {
    }

    public void feed() {
        falcon.setVoltage(FeederConstants.FEED_VOLTS);
    }

    public void unfeed() {
        falcon.setVoltage(-FeederConstants.UNFEED_VOLTS);
    }

    public void stop() {
        falcon.setVoltage(0);
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
