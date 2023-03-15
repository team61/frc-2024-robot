package frc.robot.recordings;

import java.io.File;

import com.fasterxml.jackson.databind.ObjectMapper;

public class OuterAuto {
    public static double[][] axes = new double[][]{};
    public static boolean[][] buttons = new boolean[][]{};

    public OuterAuto() {
        try {
            ObjectMapper mapper = new ObjectMapper();
            RecordingTemplate recording = mapper.readValue(new File("/home/lvuser/recording.json"), RecordingTemplate.class);

            axes = recording.axes;
            buttons = recording.buttons;
        } catch (Exception e) {
            e.printStackTrace();
        }

        System.out.println(axes.length + ", " + buttons.length);
    }
}
