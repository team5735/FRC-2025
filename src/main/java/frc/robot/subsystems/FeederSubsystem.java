package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    private final DigitalInput beamBreak = new DigitalInput(Constants.FEEDER_BEAM_ID);
    private final TalonFX falcon = new TalonFX(Constants.FEEDER_FALCON_ID);

    public FeederSubsystem() {
        falcon.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public void feed() {
        falcon.setVoltage(FeederConstants.FEED_VOLTS);
    }

    public void unfeed() {
        falcon.setVoltage(-FeederConstants.FEED_VOLTS);
    }

    public void stop() {
        falcon.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intakeSwitchStatus", getSwitchStatus());
    }

    public boolean getSwitchStatus() {
        return beamBreak.get();
    }

    public Trigger beamBreakEngaged() {
        return new Trigger(() -> !getSwitchStatus());
    }

    public Command feedCommand() {
        return startEnd(() -> feed(), () -> stop());
    }

    public Command unfeedCommand() {
        return startEnd(() -> unfeed(), () -> stop());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }

}
