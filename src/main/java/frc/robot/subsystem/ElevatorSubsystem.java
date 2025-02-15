package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.lang.System.Logger.Level;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
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
    private ElevatorConstants.Level activeLevel = ElevatorConstants.Level.BASE;

    public ElevatorSubsystem(){
        krakenLeft.setControl(new Follower(Constants.ELEVATOR_KRAKEN_RIGHT_ID, true));
        resetMeasurement();
    }

    public boolean isAtRest(){
        return (pid.getGoal() == ElevatorConstants.Level.BASE.stateSupplier.get()) && pid.atGoal();
    }

    @Override
    public void periodic(){
        pid.setGoal(activeLevel.stateSupplier.get());

        double voltsToSet = 
            pid.calculate(getPosition().in(Meters)) + ff.calculate(pid.getSetpoint().velocity);
        if(isAtRest())
            voltsToSet -= ff.getKg(); // TODO check if better logic is needed here
        
        krakenRight.setVoltage(voltsToSet);
    }

    public Distance getPosition(){
        return Feet.of(krakenRight.getPosition().getValue().in(Rotations) * ElevatorConstants.ROTATIONS_TO_FEET);
    }

    public Command toLevelCommand(ElevatorConstants.Level level){
        return runOnce(() -> setLevel(level));
    }

    private void setLevel(ElevatorConstants.Level level){
        activeLevel = level;
    }

    private void resetMeasurement(){
        resetMeasurement(Feet.of(0));
    }

    private void resetMeasurement(Distance height){
        krakenRight.setPosition(Rotations.of(height.in(Feet)/ElevatorConstants.ROTATIONS_TO_FEET));
    }
}
