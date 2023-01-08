package com.team2813.lib.motors;

public interface Motor {
    // motor control
    public void set(ControlMode controlMode, double demand);
    public void set(ControlMode controlMode, double demand, double feedForward);

    // encoder position
    public double getEncoderPosition();
    public void setEncoderPosition(double position);
    public double getVelocity();

    // config
    public void configPIDF(int slot, double p, double i, double d, double f);
    public void configPIDF(double p, double i, double d, double f);
    public void configPID(int slot, double p, double i, double d);
    public void configPID(double p, double i, double d);
    public void configMotionMagic(double maxVelocity, double maxAcceleration);
}
