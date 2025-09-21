package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Foot;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.Ports;
import frc.robot.lib.VirtualSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class Leds extends VirtualSubsystem {

    private static final int kLength = 18;
    private static final int kDefaultPercent = 75;

    private static Leds instance;

    public static Leds getInstance() {
        if (instance == null) instance = new Leds();
        return instance;
    }

    private enum LEDState {
        OFF,
        DS_DISCONNECTED,
        DISABLED,
        TELEOP,
        AUTO,
        AUTO_ALIGN,
        E_STOPPED,
        HAS_CORAL,
        MOVING_ELE;
    }

    @AutoLogOutput(key = "Leds/Current state")
    private LEDState currentState;

    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    private Leds() {
        leds = new AddressableLED(Ports.kLedPort);
        buffer = new AddressableLEDBuffer(kLength);
        leds.setLength(kLength);
        leds.setData(buffer);
        leds.start();
    }

    @Override
    public void periodic() {}

    // --------------- base functions that return an LEDPattern ---------------

    private LEDPattern rainbow() {
        return LEDPattern.rainbow(255, 128).atBrightness(Percent.of(kDefaultPercent));
    }

    private LEDPattern solidColor(Color color) {
        return LEDPattern.solid(color(color)).atBrightness(Percent.of(kDefaultPercent));
    }

    private LEDPattern conGradient(Color color1, Color color2) {
        return LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color(color1), color(color2))
                .atBrightness(Percent.of(kDefaultPercent));
    }

    private LEDPattern conGradientSeperated(Color color1, Color color2) {
        return LEDPattern.gradient(
                        LEDPattern.GradientType.kContinuous, color(color1), Color.kBlack, color(color2), Color.kBlack)
                .atBrightness(Percent.of(kDefaultPercent));
    }

    private LEDPattern disconGradient(Color color1, Color color2) {
        return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, color(color1), color(color2))
                .atBrightness(Percent.of(kDefaultPercent));
    }

    // --------------- modifiers that take an LEDPattern and change it ---------------

    private LEDPattern modifierOffset(LEDPattern base, int dist) {
        return base.offsetBy(dist);
    }

    private LEDPattern modifierScrollAbsolute(LEDPattern base, double InchesPerSecond, int density) {
        Distance ledSpacing = Foot.of(1 / density);
        return base.scrollAtAbsoluteSpeed(Inches.per(Second).of(InchesPerSecond), ledSpacing);
    }

    private LEDPattern modifierScrollRelative(LEDPattern base, double percentPerSecond) {
        return base.scrollAtRelativeSpeed(Percent.per(Second).of(percentPerSecond));
    }

    private LEDPattern modifierBreathe(LEDPattern base, double time) {
        return base.breathe(Seconds.of(time));
    }

    private LEDPattern modifierBlinkSymmetrical(LEDPattern base, double time) {
        return base.blink(Seconds.of(time));
    }

    private LEDPattern modifierBlinkAsymmetrical(LEDPattern base, double timeon, double timeoff) {
        return base.blink(Seconds.of(timeon), Seconds.of(timeoff));
    }

    private LEDPattern modifierMask(LEDPattern base, LEDPattern mask) {
        return base.mask(mask);
    }

    private LEDPattern modifierOverlay(LEDPattern base, LEDPattern mask) {
        return mask.overlayOn(base);
    }

    private LEDPattern modifierBlend(LEDPattern base, LEDPattern mask) {
        return base.blend(mask);
    }

    /** Converts a color to display the correct color (swaps green and red) */
    private Color color(Color color) {
        return new Color(color.green, color.red, color.blue);
    }
}
