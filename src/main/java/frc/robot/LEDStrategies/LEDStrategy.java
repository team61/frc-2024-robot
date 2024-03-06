// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LEDStrategies;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Utils;
import frc.robot.subsystems.LEDSystem;

public abstract class LEDStrategy {
    private LEDSystem system = LEDSystem.get();
    
    public int start, end;

    private double startTime;

    public LEDStrategy(int start, int end) {
        this.start = start;
        this.end = end;
        
        startTime = Utils.getTime();
    }

    public abstract Color getColor(int id);

    protected double getRuntime() {
        return Utils.getTime() - startTime;
    }

    protected void addToRuntime(double amount) {
        startTime -= amount;
    }
}
