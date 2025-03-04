package frc.robot.constants;

import java.awt.Color;

public class CANdleConstants {
    public enum LedState {
        FED(Color.GREEN),
        PATHING(Color.ORANGE),
        PATHEND(Color.RED);

        public final Color ledColor;

        LedState(Color ledColor) {
            this.ledColor = ledColor;
        }
    }
}