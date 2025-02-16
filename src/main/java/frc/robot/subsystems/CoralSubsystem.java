package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkFlex vortexTop = new SparkFlex(Constants.MANIPULATOR_MOTOR_BOTTOM_ID, MotorType.kBrushless);
    private final SparkFlex vortexBottom = new SparkFlex(Constants.MANIPULATOR_MOTOR_TOP_ID, MotorType.kBrushless);
    private final DigitalInput beam = new DigitalInput(Constants.INTAKE_BEAM_PIN);


    public CoralSubsystem() {
        vortexTop.configure(
            new SparkFlexConfig().inverted(false).idleMode(IdleMode.kBrake), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
        vortexBottom.configure(
            new SparkFlexConfig().inverted(true).idleMode(IdleMode.kBrake), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
        SmartDashboard.putNumber("CoralOutTopVolts", CoralConstants.TROUGH_TOP_VOLTS);
        SmartDashboard.putNumber("CoralOutBottomVolts", CoralConstants.TROUGH_BOTTOM_VOLTS);
        SmartDashboard.putNumber("CoralInTopVolts", CoralConstants.INTAKE_TOP_VOLTS);
        SmartDashboard.putNumber("CoralInBottomVolts", CoralConstants.INTAKE_BOTTOM_VOLTS);
    }

    private void intakeTop() {
        vortexTop.setVoltage(CoralConstants.INTAKE_TOP_VOLTS);
    }

    private void intakeBottom() {
        vortexBottom.setVoltage(CoralConstants.INTAKE_BOTTOM_VOLTS);
    }

    private void outtakeTop() {
       // vortexTop.setVoltage(-CoralConstants.INTAKE_TOP_VOLTS);
       vortexTop.setVoltage(-SmartDashboard.getNumber("CoralInTopVolts", CoralConstants.INTAKE_TOP_VOLTS));
    }

    private void outtakeBottom() {
        //vortexBottom.setVoltage(-CoralConstants.INTAKE_BOTTOM_VOLTS);
        vortexBottom.setVoltage(-SmartDashboard.getNumber("CoralInBottomVolts", CoralConstants.INTAKE_BOTTOM_VOLTS));
    }

    private void troughTop() {
        //vortexTop.setVoltage(CoralConstants.TROUGH_TOP_VOLTS);
        vortexTop.setVoltage(SmartDashboard.getNumber("CoralOutTopVolts", CoralConstants.TROUGH_TOP_VOLTS));
    }

    private void troughBottom() {
        //vortexBottom.setVoltage(CoralConstants.TROUGH_BOTTOM_VOLTS);
        vortexBottom.setVoltage(SmartDashboard.getNumber("CoralOutBottomVolts", CoralConstants.TROUGH_BOTTOM_VOLTS));
    }

    private void branchTop() {
        vortexTop.setVoltage(CoralConstants.BRANCH_TOP_VOLTS);
    }

    private void branchBottom() {
        vortexBottom.setVoltage(CoralConstants.BRANCH_BOTTOM_VOLTS);
    }

    private void stop() {
        vortexTop.setVoltage(0);
        vortexBottom.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intakeSwitchStatus", getSwitchStatus());
    }

    public boolean getSwitchStatus() {
        return beam.get();
    }

    public Trigger beamBreakEngaged() {
        return new Trigger(() -> !getSwitchStatus());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }

    public Command feedInCommand() {
        // return runOnce(() -> intakeBottom())
        //     .withDeadline(new WaitCommand(CoralConstants.FEED_DELAY_SECONDS))
        //     .andThen(runOnce(() -> intakeTop()))
        //     .until(beamBreakEngaged())
        //     .finallyDo(() -> stop());
        return new FunctionalCommand(() -> {
            intakeTop();
            intakeBottom();
        }, () -> {}, (b) -> stop(), () -> false, this);

    }

    public Command troughCommand() {
        return new FunctionalCommand(() -> {
            troughTop();
            troughBottom();
        }, () -> {}, (b) -> stop(), () -> false, this);
    }

    public Command branchCommand() {
        return new FunctionalCommand(() -> {
            branchTop();
            branchBottom();
        }, () -> {}, (b) -> stop(), () -> false, this);
    }

    public Command outtakeCommand() {
        return new FunctionalCommand(() -> {
            outtakeTop();
            outtakeBottom();
        }, () -> {}, (b) -> stop(), () -> false, this);
    }
}