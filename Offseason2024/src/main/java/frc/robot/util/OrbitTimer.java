package frc.robot.util;

public class OrbitTimer {

    private long startTime;

    public OrbitTimer() {
        this.startTime = System.currentTimeMillis();
    }

    public void start() {
        this.startTime = System.currentTimeMillis();
    }

    public long getTimeDeltaMillis() {
        return System.currentTimeMillis() - this.startTime;
    }

    // public double getTimeDeltaSec() {
    //     double deltaT = (System.currentTimeMillis() - this.startTime) / 1000.0;
    //     return deltaT;
    // }                                                                                   < this was a little more messy than it needed to be so use the method below instead

    public double getTimeDeltaSec() {
        return getTimeDeltaMillis() / 1000.0;
    }

}