// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystemHelpers;

/** Add your docs here. */
public class ThrottlePreset {
    public double translationFactor;
    public double rotationFactor;

    public ThrottlePreset(double translationFactor, double rotationFactor) {
        this.translationFactor = translationFactor;
        this.rotationFactor = rotationFactor;
    }
}
