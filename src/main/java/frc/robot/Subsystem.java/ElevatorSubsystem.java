package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax sparkMax_feed = new SparkMax(Constants.ALGAE_FALCON_ID, MotorType.kBrushless);
    private final SparkMaxConfig pull_config = new SparkMaxConfig();

    public ElevatorSubsystem() {
        sparkMax_feed.configure(pull_config.inverted(true), null, null);
    }

    public void feed() {
        sparkMax_feed.setVoltage(ElevatorConstants.ELEVATOR_MOTOR_LEFT_VOLTS);
    }

    public void spit() {
        sparkMax_feed.setVoltage(-ElevatorConstants.ELEVATOR_MOTOR_LEFT_VOLTS);
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
