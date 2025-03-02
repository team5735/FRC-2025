package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralConstants;
import frc.robot.util.TunableNumber;

public class CoralSubsystem extends SubsystemBase {
    private final SparkFlex vortexTop = new SparkFlex(Constants.CORAL_MOTOR_BOTTOM_ID, MotorType.kBrushless);
    private final SparkFlex vortexBottom = new SparkFlex(Constants.CORAL_MOTOR_TOP_ID, MotorType.kBrushless);

    private final SparkMax ejector = new SparkMax(Constants.CORAL_EJECTOR_ID, MotorType.kBrushless);

    private final DigitalInput beam = new DigitalInput(Constants.INTAKE_BEAM_PIN);

    private final TalonFX falcon = new TalonFX(Constants.FEEDER_FALCON_ID);

    private TunableNumber outTopVolts = new TunableNumber("coral", "out_top_volts", CoralConstants.TROUGH_TOP_VOLTS);
    private TunableNumber outBottomVolts = new TunableNumber("coral", "out_bottom_volts",
            CoralConstants.TROUGH_BOTTOM_VOLTS);
    private TunableNumber inTopVolts = new TunableNumber("coral", "in_top_volts", CoralConstants.INTAKE_TOP_VOLTS);
    private TunableNumber inBottomVolts = new TunableNumber("coral", "in_bottom_volts",
            CoralConstants.INTAKE_BOTTOM_VOLTS);

    private TunableNumber ejectorVolts = new TunableNumber("coral", "ejector_volts", CoralConstants.EJECTOR_VOLTS);

    private TunableNumber feederVolts = new TunableNumber("feeder", "feeder_volts", CoralConstants.FEEDER_VOLTS);

    public CoralSubsystem() {
        vortexTop.configure(
                new SparkFlexConfig().inverted(false).idleMode(IdleMode.kBrake),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        vortexBottom.configure(
                new SparkFlexConfig().inverted(true).idleMode(IdleMode.kBrake),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        ejector.configure(
                new SparkMaxConfig().idleMode(IdleMode.kBrake),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        falcon.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    private void intakeTop() {
        vortexTop.setVoltage(inTopVolts.get());
    }

    private void intakeBottom() {
        vortexBottom.setVoltage(inBottomVolts.get());
    }

    private void outtakeTop() {
        vortexTop.setVoltage(-inTopVolts.get());
    }

    private void outtakeBottom() {
        vortexBottom.setVoltage(-inBottomVolts.get());
    }

    private void troughTop() {
        vortexTop.setVoltage(outTopVolts.get());
    }

    private void troughBottom() {
        vortexBottom.setVoltage(outBottomVolts.get());
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

    private void ejectOut() {
        ejector.setVoltage(ejectorVolts.get());
    }

    private void ejectResetPose() {
        ejector.setVoltage(-ejectorVolts.get());
    }

    private void stopEject() {
        ejector.setVoltage(0);
    }

    public void feed() {
        falcon.setVoltage(feederVolts.get());
    }

    public void stopFeed() {
        falcon.setVoltage(0);
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

    // Manipulator feeds out
    public Command simpleFeedCommand() {
        return startEnd(() -> {
            intakeTop();
            intakeBottom();
        }, () -> stop());
    }

    public Command simpleEjectOutCommand() {
        return startEnd(() -> {
            intakeBottom();
            ejectOut();
        }, () -> {
            stop();
            stopEject();
        });
    }

    public Command simpleEjectResetCommand() {
        return startEnd(() -> ejectResetPose(),
                () -> stopEject());
    }

    // TODO: implement beam break in hardware & find proper delay
    public Command feedStageCommand() {
        return runOnce(() -> intakeBottom())
                .withDeadline(new WaitCommand(CoralConstants.FEED_DELAY_SECONDS))
                .andThen(runOnce(() -> intakeTop()))
                .until(beamBreakEngaged())
                .finallyDo(() -> stop());
    }

    public Command troughCommand() {
        return startEnd(() -> {
            troughTop();
            troughBottom();
        }, () -> stop());
    }

    public Command branchCommand() {
        return startEnd(() -> {
            branchTop();
            branchBottom();
        }, () -> stop());
    }

    public Command l4BranchCommand() {
        return branchCommand().withTimeout(CoralConstants.L4_EJECTION_TIMEOUT).andThen(startEnd(() -> {
            ejectOut();
        }, () -> {
            stopEject();
        }));
    }

    public Command outtakeCommand() {
        return startEnd(() -> {
            outtakeTop();
            outtakeBottom();
        }, () -> stop());
    }
}
