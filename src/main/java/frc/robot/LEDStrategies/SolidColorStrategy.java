// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LEDStrategies;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class SolidColorStrategy extends LEDStrategy {
    Color color;

    public SolidColorStrategy(int start, int end, Color color) {
        super(start, end);

        this.color = color;
    }

    @Override
    public Color getColor(int id) {
        return color;
    }
}