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
    public void execute() {
        //runs repeatedly until finished
    }

    @Override
    public boolean isFinished() {
        //called repeatedly. command stops if it even returns true
        return true;
    }
}
