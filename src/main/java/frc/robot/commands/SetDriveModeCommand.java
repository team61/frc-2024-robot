package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.*;
import static frc.robot.Globals.*;

public class SetDriveModeCommand extends CommandBase {
    private final String mode;

    public SetDriveModeCommand(String newMode) {
        mode = newMode;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        CURRENT_DRIVE_MODE = mode;

        if (CURRENT_DRIVE_MODE.equals(TANK_DRIVE)) {
            USE_OLD_SWERVE_DRIVE = true;
        } else {
            USE_OLD_SWERVE_DRIVE = false;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
