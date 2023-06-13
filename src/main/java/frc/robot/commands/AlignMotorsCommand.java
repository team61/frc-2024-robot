package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OldSwerveDriveSubsystem;

import static frc.robot.Constants.*;
import static frc.robot.Globals.*;

public class AlignMotorsCommand extends CommandBase {
    private final OldSwerveDriveSubsystem swervedrive;
    private final String[] directions;
    private final int maxTime;
    private long startTime;
    private boolean finished = false;

    public AlignMotorsCommand(OldSwerveDriveSubsystem sd, int maxMillis, String dir1, String dir2, String dir3, String dir4) {
        swervedrive = sd;
        directions = new String[] { dir1, dir2, dir3, dir4 };
        maxTime = maxMillis;
        
        addRequirements(sd);
    }

    public AlignMotorsCommand(OldSwerveDriveSubsystem sd, String dir, int maxMillis) {
        swervedrive = sd;
        directions = new String[] { dir, dir, dir, dir };
        maxTime = maxMillis;
        
        addRequirements(sd);
    }

    public AlignMotorsCommand(OldSwerveDriveSubsystem sd, String dir) {
        swervedrive = sd;
        directions = new String[] { dir, dir, dir, dir };
        maxTime = 500;
        
        addRequirements(sd);
    }
    
    @Override
    public void initialize() {
        if (!USE_OLD_SWERVE_DRIVE) return;
        // swervedrive.disableBreaks();
        startTime = System.currentTimeMillis();
        if (directions[0] == DIAGONAL) {
            IS_ROTATING = true;
        } else {
            IS_ROTATING = false;
        }
    }

    @Override
    public void execute() {
        if (!USE_OLD_SWERVE_DRIVE) {
            finished = true;
            return;
        }

        CURRENT_DIRECTIONS = directions;
        if (CURRENT_DRIVE_MODE == TANK_DRIVE) return;
        
        finished = swervedrive.alignMotors(directions);
    }

    @Override
    public void end(boolean interrupted) {
        // swervedrive.enableBreaks();
        swervedrive.setRotationVoltage(0);
        finished = false;
    }

    @Override
    public boolean isFinished() {
        if (CURRENT_DRIVE_MODE == TANK_DRIVE) return true;
        
        boolean hasMaxTimePassed = System.currentTimeMillis() - startTime > maxTime;
        return finished || hasMaxTimePassed;
    }
}
