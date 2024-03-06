// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LEDStrategies;

/** Add your docs here. */
public abstract class LEDStrategyDecorator extends LEDStrategy {
    protected LEDStrategy child;

    public LEDStrategyDecorator(LEDStrategy child) {
        super(child.start, child.end);
        
        this.child = child;
    }
}
