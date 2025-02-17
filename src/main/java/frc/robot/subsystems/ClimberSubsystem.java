package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ClimberSubsystem extends SubsystemBase{
    private final TalonFX falconRight = new TalonFX(Constants.CLIMBER_FALCON_RIGHT_ID);
    private final TalonFX falconLeft = new TalonFX(Constants.CLIMBER_FALCON_LEFT_ID);

    public ClimberSubsystem(){
    }
}
