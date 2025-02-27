package frc.robot.constants;

import java.awt.Color;

public class CANdleConstants {
    public enum LedState {
        READY(Color.BLUE),
        AUTO(new Color(127, 0, 127)), // Purple
        AIMING(new Color(127, 127, 0)), // Yellow-Green
        AIMED(Color.GREEN),
        INTAKE_RUNNING(new Color(127, 0, 0)), // Dim Red
        SHOOTING(Color.RED);

        public final Color ledColor;
        
        LedState(Color ledColor){
            this.ledColor = ledColor;
        }
    }
}