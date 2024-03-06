// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LEDStrategies;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class BlinkDecorator extends LEDStrategyDecorator {
    private double cycleTime, offset, duty;

    public BlinkDecorator(LEDStrategy child, double cycleTime) {
        super(child);

        this.cycleTime = cycleTime;
        this.offset = 0;
        this.duty = 0.5;
    }

    public BlinkDecorator(LEDStrategy child, double cycleTime, double offset) {
        super(child);

        this.cycleTime = cycleTime;
        this.offset = offset;
        this.duty = 0.5;
    }
    
    public BlinkDecorator(LEDStrategy child, double cycleTime, double offset, double duty) {
        super(child);

        this.cycleTime = cycleTime;
        this.offset = offset;
        this.duty = duty;
    }
    
    @Override
    public Color getColor(int id) {
        double cycleProgress = ((getRuntime() + offset) % cycleTime) / cycleTime;

        if (cycleProgress < duty) {
            return child.getColor(id);
        }
        else {
            return Color.kBlack;
        }
    }
}
