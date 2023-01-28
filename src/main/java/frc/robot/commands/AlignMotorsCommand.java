package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AlignMotorsCommand extends CommandBase {
    private final SwerveDriveSubsystem swervedrive;
    private final String direction;
    private long startTime;
    private boolean finished = false;

    public AlignMotorsCommand(SwerveDriveSubsystem sd, String dir) {
        swervedrive = sd;
        direction = dir;
        
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
        return finished || (System.currentTimeMillis() - startTime > 500);
    }
}
