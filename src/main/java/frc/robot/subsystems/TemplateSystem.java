package frc.robot.subsystems;

public class TemplateSystem {
    private static TemplateSystem system;

    //properties

    private TemplateSystem() {
        //initialize properties
    }

    public static TemplateSystem get() {
        if (system == null) {
            system = new TemplateSystem();
        }

        return system;
    }

    //methods
}
