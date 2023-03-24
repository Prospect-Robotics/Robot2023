package com.team2813.frc2023.util;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2813.frc2023.Constants.*;

/*
ChannelA is green
ChannelB is red
ChannelC is blue
*/
public class Lightshow extends SubsystemBase {

    private final CANifier canifier;

    public Lightshow() {
        canifier = new CANifier(CANIFIER_ID);
        setLight(Light.DEFAULT);
    }

    public void setLight(int r, int g, int b) {
        canifier.setLEDOutput(r / 255.0, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(g / 255.0, CANifier.LEDChannel.LEDChannelA);
        canifier.setLEDOutput(b / 255.0, CANifier.LEDChannel.LEDChannelC);
    }

    public void setLight(Light light) {
//        if(light.flash){
//            flashLight = new BlinkLightsCommand(light, light.period);
//            flashLight.schedule();
//        }
//        else {
//            if (flashLight != null) flashLight.cancel();
//            setLight(light.r, light.g, light.b);
//        }
        setLight(light.r, light.g, light.b);
    }

//    public void setDefaultLight(Light light){
//        flashLight = new BlinkLightsCommand(light.period);
//        flashLight.schedule();
//    }


    public enum Light {
        DEFAULT(255, 255, 255,true, 0.0), //white
        ENABLED(65, 105, 225,true, 0.5), //royal blue
        DISABLED(255, 0, 0,false, 0.0), //red
        AUTONOMOUS(255, 0, 255,true, 0.75), //flashing purple
        AUTO_BALANCE(0, 0, 128,true, 0.5), //flashing navy blue
        PICKUP(255, 111, 0,false, 0.0); //orange


        final int r;
        final int g;
        final int b;
        final boolean flash;

        final double period;

        Light(int r, int g, int b, boolean flash, Double period) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.flash = flash;
            this.period = period;
        }
    }
}
