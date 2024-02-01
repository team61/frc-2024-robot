// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

/** Add your docs here. */
public class Utils {
    public static double getTime() {
        return Instant.now().toEpochMilli() / 1000.0;
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static double round(double a) {
        return Math.round(a * 100) / 100.0;
    }
}
