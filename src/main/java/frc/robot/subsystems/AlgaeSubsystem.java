package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.Constants;

public class AlgaeSubsystem extends SubsystemBase {
    private final TalonFX falcon = new TalonFX(Constants.ALGAE_FALCON_ID);

    public AlgaeSubsystem() {
        falcon.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public void grab() {
        falcon.setVoltage(AlgaeConstants.GRAB_VOLTS);
    }

    public void spit() {
        falcon.setVoltage(-AlgaeConstants.SPIT_VOLTS);
    }

    public void stop() {
        falcon.setVoltage(0);
    }

    public Command grabStopCommand() {
        return startEnd(() -> grab(), () -> stop());
    }

    public Command spitStopCommand() {
        return startEnd(() -> spit(), () -> stop());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }
}
