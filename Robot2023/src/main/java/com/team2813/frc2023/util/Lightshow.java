package com.team2813.frc.util;

import com.ctre.phoenix.CANifier;

/*
ChannelA is green
ChannelB is red
ChannelC is blue
*/
public class Lightshow {

    private final CANifier canifier;

    public Lightshow(int canifierID) {
        canifier = new CANifier(canifierID);
        setLight(Light.DEFAULT);
    }

    public void setLight(int r, int g, int b) {
        canifier.setLEDOutput(r / 255.0, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(g / 255.0, CANifier.LEDChannel.LEDChannelA);
        canifier.setLEDOutput(b / 255.0, CANifier.LEDChannel.LEDChannelC);
    }

    public void setLight(Light light) {
        setLight(light.r, light.g, light.b);
    }

    public enum Light {
        DEFAULT(255, 255, 255),
        ENABLED(0, 255, 0),
        DISABLED(255, 0, 0),
        AUTONOMOUS(255, 0, 255),
        READY_TO_AUTO_SHOOT(0, 0, 255),
        READY_TO_MANUAL_SHOOT(255, 105, 180),
        SPOOLING(255, 255, 0),
        CLIMBING(0, 0, 128);

        final int r;
        final int g;
        final int b;

        Light(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }
}