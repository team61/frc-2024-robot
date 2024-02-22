package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LauncherSystem;

public class ReadyLauncherCommand extends CommandBase {    
    public static ReadyLauncherCommand speakerCommand = new ReadyLauncherCommand(Constants.launcherSpeakerPower);
    
    private LauncherSystem launcherSystem = LauncherSystem.get();

    private double power;

    public ReadyLauncherCommand(double power) {
        this.power = power;
    }

    @Override
    public void initialize() {
        launcherSystem.ready(power);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
