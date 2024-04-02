// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LEDStrategies;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class AsciiStrategy extends LEDStrategy {
    public String text;
    public Color color;

    public AsciiStrategy(int start, int end, String text, Color color) {
        super(start, end);

        this.text = text;
        this.color = color;
    }

    public Color getColor(int id) {
        int bitInText = id % (text.length() * 8);
        int bitInChar = bitInText % 8;
        int character = (int)text.charAt((int)Math.floor(bitInText / 8));
        boolean bit = ((character >> bitInChar) & 1) == 1;

        if (bit) {
            return color;
        }
        else {
            return Color.kBlack;
        }
    }
}
