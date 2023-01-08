package com.team2813.frc2023.util;

public class Units2813 {
    public static double ticksToMotorRevs(double ticks, int cpr) {
        return ticks / cpr;
    }

    public static int motorRevsToTicks(double revs, int cpr) {
        return (int) revs * cpr;
    }

    public static double motorRevsToWheelRevs(double revs, double gearRatio) {
        return revs * gearRatio;
    }

    public static double wheelRevsToMotorRevs(double revs, double gearRatio) {
        return revs / gearRatio;
    }
}
