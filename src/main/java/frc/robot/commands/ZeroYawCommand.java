package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroYawCommand extends CommandBase {
    private final AHRS gyro;
    public ZeroYawCommand(AHRS g) {
        gyro = g;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        gyro.zeroYaw();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
