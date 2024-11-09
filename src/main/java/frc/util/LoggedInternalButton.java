package frc.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.button.InternalButton;

public class LoggedInternalButton extends InternalButton {
    private final String key;
    
    public LoggedInternalButton(String key) {
        this.key = key;
        setPressed(false);
    }

    @Override
    public void setPressed(boolean pressed) {
        super.setPressed(pressed);
        Logger.recordOutput(key, pressed);
    }
}
