package frc.robot.subsystems;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Path;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Sequence;
import javax.sound.midi.Sequencer;
import javax.sound.midi.ShortMessage;

import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystemHelpers.HardwareMidiChannel;

public class AudioSystem {
    private static AudioSystem system;

    private HardwareMidiChannel[] channels;

    private AudioSystem() {
        //initialize properties
    }

    public static AudioSystem get() {
        if (system == null) {
            system = new AudioSystem();
        }

        return system;
    }

    public void startMidi(String name) {
        FileInputStream stream = null;
        
        try {
            //setup i/o
        
            String path = "C:\\Users\\bvtro\\projects\\robots\\frc-2024-noah-test\\src\\main\\java\\frc\\robot\\midi\\" + name;
            stream = new FileInputStream(path);
            Sequencer sequencer = MidiSystem.getSequencer();
            sequencer.setSequence(stream);
            Sequence sequence = sequencer.getSequence();

            //setup channels

            double startTime = Utils.getTime();
            int trackCount = sequence.getTracks().length;
            double tickLength = sequence.getTickLength();

            channels = new HardwareMidiChannel[trackCount];

            for (int i = 0; i < trackCount; i++) {
                channels[i] = new HardwareMidiChannel(Constants.audioMotorNumbers[i], sequence.getTracks()[i], tickLength);
                channels[i].startPlaying(startTime);
            }
        }
        catch (FileNotFoundException e) {
            System.out.println("FileNotFoundException");
        }
        catch (MidiUnavailableException e) {
            System.out.println("MidiUnavailableException");
        }
        catch (IOException e) {
            System.out.println("IOException");
        }
        catch (InvalidMidiDataException e) {
            System.out.println("InvalidMidiDataException");
        }
        finally {
            if (stream != null) {
                try {
                    stream.close();
                }
                catch (IOException e) {
                    System.out.println("IOException on closing stream");
                }
            }
        }
    }

    public void update() {
        if (channels != null) {
            for (HardwareMidiChannel channel : channels) {
                channel.update();
            }
        }
    }

    public void stop() {
        for (HardwareMidiChannel channel : channels) {
            channel.stop();
        }
    }

    //methods
}
