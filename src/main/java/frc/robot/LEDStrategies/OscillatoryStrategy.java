// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LEDStrategies;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class OscillatoryStrategy extends LEDStrategy {
    public OscillatoryStrategy(int start, int end) {
        super(start, end);
    }

    @Override
    public Color getColor(int id) {
        double t = getRuntime();
        t += 0.02 * id;
        t += 1 * Math.sin(0.05 * id + 0.5 * t);
        double hue = Math.sin(1.5 / Math.sin(10 * t));

        Color color = Color.kPurple;
        if (hue < -0.7) {
            color = Color.kGold;
        }
        return color;
    }
}
