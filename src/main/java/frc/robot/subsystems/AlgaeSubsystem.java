package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.Constants;

public class AlgaeSubsystem extends SubsystemBase {
    private final TalonFX falcon = new TalonFX(Constants.ALGAE_FALCON_ID);

    public AlgaeSubsystem() {
    }

    public void feed() {
        falcon.setVoltage(AlgaeConstants.GRAB_VOLTS);
    }

    public void spit() {
        falcon.setVoltage(-AlgaeConstants.GRAB_VOLTS);
    }

    public void stop() {
        falcon.setVoltage(0);
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
