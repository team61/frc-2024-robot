package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.LEDStrategies.LEDStrategy;

public class LEDSystem {
    private static LEDSystem system;

    public LEDStrategy[] strategies;

    private AddressableLED strip;
    private AddressableLEDBuffer buffer;

    private LEDSystem() {
        strip = new AddressableLED(Constants.ledPort);
        buffer = new AddressableLEDBuffer(Constants.ledLength);

        strip.setLength(Constants.ledLength);
        strip.start();
    }

    public static LEDSystem get() {
        if (system == null) {
            system = new LEDSystem();
        }

        return system;
    }

    public void set(int id, Color color) {
        buffer.setLED(id, color);
    }

    public void setRange(int start, int end, Color color) {
        for (int i = start; i < end; i++) {
            set(i, color);
        }
    }

    public void setAll(Color color) {
        setRange(0, buffer.getLength(), color);
    }

    public void refresh() {
        strip.setData(buffer);
    }

    public void update() {
        setAll(Color.kBlack);

        for (LEDStrategy strategy : strategies) {
            for (int i = strategy.start; i < strategy.end; i++) {
                set(i, strategy.getColor(i));
            }
        }

        refresh();
    }
}
