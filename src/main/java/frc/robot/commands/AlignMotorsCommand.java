package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.*;
import static frc.robot.Globals.*;

public class AlignMotorsCommand extends CommandBase {
    private final SwerveDriveSubsystem swervedrive;
    private final String direction;
    private final int maxTime;
    private long startTime;
    private boolean finished = false;

    public AlignMotorsCommand(SwerveDriveSubsystem sd, String dir, int maxMillis) {
        swervedrive = sd;
        direction = dir;
        maxTime = maxMillis;
        
        addRequirements(sd);
    }

    public AlignMotorsCommand(SwerveDriveSubsystem sd, String dir) {
        swervedrive = sd;
        direction = dir;
        maxTime = 500;
        
        addRequirements(sd);
    }
    
    @Override
    public void initialize() {
        swervedrive.disableBreaks();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        finished = swervedrive.alignMotors(direction);
    }

    @Override
    public void end(boolean interrupted) {
        swervedrive.enableBreaks();
        swervedrive.setRotationVoltage(0);
    }

    @Override
    public boolean isFinished() {
        if (CURRENT_DRIVE_MODE == TANK_DRIVE) return false;
        
        boolean hasMaxTimePassed = System.currentTimeMillis() - startTime > maxTime;
        return finished || hasMaxTimePassed;
    }
}
