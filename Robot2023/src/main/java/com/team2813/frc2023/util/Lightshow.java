package com.team2813.frc2023.util;

import com.ctre.phoenix.CANifier;
import com.team2813.frc2023.commands.BlinkLightsCommand;

import edu.wpi.first.wpilibj2.command.Command;
/*
ChannelA is green
ChannelB is red
ChannelC is blue
*/
public class Lightshow {
public boolean flashing = false; 
public Command BlinkLight; 
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
        if (BlinkLight != null){
            BlinkLight.cancel();
        }

        if(light.flash){
            BlinkLight =  new BlinkLightsCommand(light, 1.0); 
        }
        else{
            setLight(light.r , light.g, light.b);
        }
        
    }
    

    public enum Light {
        DEFAULT(255, 255, 255,false), //white
        ENABLED(0, 255, 0,false), //green
        DISABLED(255, 0, 0,false), //red
        AUTONOMOUS(255, 0, 255,true), //purple
        AutoBalance(0, 0, 128,true), //navy blue
        PICKUP(255, 111, 0,false); //orange


        final int r;
        final int g;
        final int b;
        final boolean flash;

        Light(int r, int g, int b,  boolean flash) {
            this.flash = flash;
            this.r = r;
            this.g = g;
            this.b = b;

        }
    }
}