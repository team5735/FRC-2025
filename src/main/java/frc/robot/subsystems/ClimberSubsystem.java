package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX falconRight = new TalonFX(Constants.CLIMBER_FALCON_RIGHT_ID);
    private final TalonFX falconLeft = new TalonFX(Constants.CLIMBER_FALCON_LEFT_ID);

    public ClimberSubsystem() {
        falconRight.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        falconRight.setPosition(Rotations.of(0));
        falconLeft.setControl(new Follower(Constants.CLIMBER_FALCON_RIGHT_ID, true));
    }

    public Angle getPosition() {
        return falconRight.getPosition().getValue();
    }

    private void out() {
        falconRight.setVoltage(ClimberConstants.OUT_VOLTS);
    }

    private void climb() {
        falconRight.setVoltage(ClimberConstants.CLIMB_VOLTS);
    }

    private void stop() {
        falconRight.setVoltage(0);
    }

    public Command outCommand() {
        return startEnd(() -> out(), () -> stop()).until(() -> getPosition().equals(ClimberConstants.OUTER_LIMIT));
    }

    public Command climbCommand() {
        return startEnd(() -> climb(), () -> stop()).until(() -> getPosition().equals(ClimberConstants.INNER_LIMIT));
    }
}
