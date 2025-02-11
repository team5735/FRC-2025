package frc.robot.subsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX krakenRight = new TalonFX(Constants.ELEVATOR_KRAKEN_RIGHT_ID);
    private final TalonFX krakenLeft = new TalonFX(Constants.ELEVATOR_KRAKEN_LEFT_ID);
    private ProfiledPIDController pid = new ProfiledPIDController(
        ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD,
        new Constraints(
            ElevatorConstants.MAX_VELOCITY.in(MetersPerSecond), 
            ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond)
        )
    );
    private ElevatorFeedforward ff = new ElevatorFeedforward(
        ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA
    );

    public ElevatorSubsystem(){
        krakenLeft.setControl(new Follower(Constants.ELEVATOR_KRAKEN_RIGHT_ID, true));
    }

    @Override
    public void periodic(){

    }
}
