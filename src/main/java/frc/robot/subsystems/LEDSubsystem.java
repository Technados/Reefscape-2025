package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class LEDSubsystem extends SubsystemBase {
    private final Spark blinkin;
    private double defaultPattern = -0.51; // Rainbow palette (or choose your favorite)
    private boolean overriding = false;
    private double overrideEndTime = 0;

    public LEDSubsystem(int pwmPort) {
        blinkin = new Spark(pwmPort);
        blinkin.set(defaultPattern);
    }

    public void setDefaultPattern(double pattern) {
        defaultPattern = pattern;
        if (!overriding) {
            blinkin.set(pattern);
        }
    }

    public void setPattern(double pattern) {
        blinkin.set(pattern);
        overriding = false;
    }

    public void flashPattern(double pattern, double duration) {
        blinkin.set(pattern);
        overriding = true;
        overrideEndTime = Timer.getFPGATimestamp() + duration;
    }

    public void resetToDefault() {
        blinkin.set(defaultPattern);
        overriding = false;
    }

    @Override
    public void periodic() {
        if (overriding && Timer.getFPGATimestamp() >= overrideEndTime) {
            resetToDefault();
        }
    }
}
