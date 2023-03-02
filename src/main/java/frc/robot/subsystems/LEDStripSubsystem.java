package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStripSubsystem extends SubsystemBase {
    private final AddressableLED strip;
    private final AddressableLEDBuffer buffer;

    public LEDStripSubsystem(int port, int length) {
        strip = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);

        strip.setLength(buffer.getLength());
        strip.start();
    }

    public void setRGB(int index, int r, int g, int b) {
        buffer.setRGB(index, r, g, b);
    }

    public void setStripRGB(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            setRGB(i, r, g, b);
        }
    }

    public void off() {
        setStripRGB(0, 0, 0);
    }

    public int getLength() {
        return buffer.getLength();
    }

    @Override
    public void periodic() {
        strip.setData(buffer);
    }
}
