// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystemHelpers;

/** Add your docs here. */
public class AngleMotorWrapper extends TargettedMotorWrapper {
    public AngleMotorWrapper(int[] motorNumbers, boolean[] inverted) {
        super(motorNumbers, inverted);
    }
    
    @Override
    public void update() {
        double position = getPosition();
        
        while (position - targetPosition > 180) {
            targetPosition += 360;
        }

        while (position - targetPosition < -180) {
            targetPosition -= 360;
        }

        super.update();
    }
}
