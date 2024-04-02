package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystemHelpers.ArmPreset;
import frc.robot.subsystems.ArmSystem;

public class TargetArmSystemCommand extends CommandBase {    
    public static TargetArmSystemCommand pickupPresetCommand = new TargetArmSystemCommand(Constants.pickupPreset);
    public static TargetArmSystemCommand ampPresetCommand = new TargetArmSystemCommand(Constants.ampPreset);
    public static TargetArmSystemCommand homePresetCommand = new TargetArmSystemCommand(Constants.homePreset);
    public static TargetArmSystemCommand stageStartPresetCommand = new TargetArmSystemCommand(Constants.stageStartPreset);
    public static TargetArmSystemCommand removePresetCommand = new TargetArmSystemCommand(null);

    ArmSystem armSystem = ArmSystem.get();
    
    ArmPreset preset;

    public TargetArmSystemCommand(ArmPreset preset) {
        this.preset = preset;
    }

    @Override
    public void initialize() {
        armSystem.setPreset(preset);
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
