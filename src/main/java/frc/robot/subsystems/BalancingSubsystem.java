package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.*;

public class BalancingSubsystem extends SubsystemBase {
    private final AHRS gyro;
    private final DriveTrain drivetrain;
    private boolean enabled = false;

    public BalancingSubsystem(AHRS g, DriveTrain dt) {
        gyro = g;
        drivetrain = dt;
    }

    public void enable() {
        enabled = true;
        drivetrain.disableWheelBreaks();
    }

    public void disable() {
        enabled = false;
        drivetrain.enableWheelBreaks();
    }

    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void periodic() {
        if (!enabled) return;

        // double newVoltage = -gyro.getRate() * 10;
        // double volts = clamp(newVoltage, -1, 1);
        // if (volts < 0.1 && volts > -0.1) {
        //     drivetrain.tankdrive.stop();
        // } else {
        //     drivetrain.tankdrive.driveVolts(volts, volts);
        // }
        
        double pitch = Math.round(gyro.getPitch() / 5) * 5;
        double speed = -pitch * 0.0085;
        double error = gyro.getRate();
        drivetrain.tankdrive.driveSpeed(speed + error * Math.abs(error / 4), speed - error * Math.abs(error));
        drivetrain.swervedrive.alignMotors(FORWARDS);
    }
}
