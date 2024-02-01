// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystemHelpers;

import javax.sound.midi.MidiMessage;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Track;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Utils;

public class HardwareMidiChannel {
    private TalonFX motor;
    private Track track;
    private double startTime;
    private double tickLength;

    public HardwareMidiChannel(int motorNumber, Track track, double tickLength) {
        motor = new TalonFX(motorNumber);
        this.track = track;
        this.tickLength = tickLength;
    }

    public void startPlaying(double startTime) {
        this.startTime = startTime;
    }

    public void update() {
        if (Utils.getTime() - startTime >= track.get(0).getTick() * tickLength) {
            MidiMessage message = track.get(0).getMessage();
            
            if (message instanceof ShortMessage) {
                ShortMessage shortMessage = (ShortMessage) message;

                if (shortMessage.getCommand() == ShortMessage.NOTE_ON) {
                    play(shortMessage.getData1());
                }
                else if (shortMessage.getCommand() == ShortMessage.NOTE_OFF) {
                    stop();
                }
            }

            track.remove(track.get(0));
        }
    }

    public void play(int noteNumber) {
        double freq = Math.pow(2, (noteNumber - 69) / 12) * 440;
        motor.set(ControlMode.MusicTone, freq);
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }
}
