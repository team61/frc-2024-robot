package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandTemplate extends CommandBase {    
    //properties

    public CommandTemplate() {
        //constructor
    }

    @Override
    public void initialize() {
        //called once at beginning
    }

    @Override
    public void execute() {
        //called repeatedly during execution
    }

    @Override
    public void end(boolean interrupted) {
        //called once at end
    }

    @Override
    public boolean isFinished() {
        //called repeatedly. command stops if it ever returns true
        return true;
    }
}
