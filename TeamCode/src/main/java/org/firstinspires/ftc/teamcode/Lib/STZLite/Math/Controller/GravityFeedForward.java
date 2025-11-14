package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.Gains.ProfileGains;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.Helpers.Output;

public class GravityFeedForward {
    private  ProfileGains.GravityFeedForwardGains gains;

    public GravityFeedForward(ProfileGains.GravityFeedForwardGains gains) {
        this.gains = gains;
    }

    public GravityFeedForward(double kS, double kV, double kG, double kA) {
        this(new ProfileGains.GravityFeedForwardGains(kS, kG, kV, kA));
    }

    public GravityFeedForward(double kS, double kV, double kG) {
        this(kS, kV, kG,0);
    }

    public ProfileGains.GravityFeedForwardGains getGains() {
        return gains;
    }

    public void setGains(ProfileGains.GravityFeedForwardGains gains) {
        this.gains = gains;
    }

    public Output calculate(double velocity, double acceleration){
        return ()-> Math.signum(velocity) * gains.kS() +gains.kG() +gains.kV() * velocity +gains.kA() * acceleration;
    }

}
