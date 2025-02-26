

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.Color;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANdleConstants;
import frc.robot.constants.Constants;

public class CANdleSubsystem extends SubsystemBase {
    private CANdle candle;

    public CANdleSubsystem() {
        candle = new CANdle(Constants.CANDLE_ID);

        candle.configLEDType(LEDStripType.GRB);
    }

    public void setIdle() {
        candle.animate(new RainbowAnimation(0.5, 1.0, 8));
    }

    public Command colorReadyCommand() {
        return setToColorByState(CANdleConstants.LedState.READY);
    }

    public Command colorAutoCommand() {
        return setToColorByState(CANdleConstants.LedState.AUTO);
    }

    public Command colorAimingCommand() {
        return setToColorByState(CANdleConstants.LedState.AIMING);
    }

    public Command colorAimedCommand() {
        return setToColorByState(CANdleConstants.LedState.AIMED);
    }

    public Command colorIntakeRunningCommand() {
        return setToColorByState(CANdleConstants.LedState.INTAKE_RUNNING);
    }

    public Command colorShootingCommand() {
        return setToColorByState(CANdleConstants.LedState.SHOOTING);
    }

    private Command setToColorByState(CANdleConstants.LedState state) {
        return runOnce(() -> {
            setToColor(state.ledColor);
        });
    }

    private void setToColor(Color color) {
        candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    }
}