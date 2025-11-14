package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.Gains.ProfileGains;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.Helpers.Output;

public class FeedForwardController {

    private ProfileGains.FeedForwardGains gains;

    public  FeedForwardController(ProfileGains.FeedForwardGains gains){
        this.gains = gains;
    }

    public FeedForwardController(double kS, double kV, double kA){
        this(new ProfileGains.FeedForwardGains(kS, kV, kA));
    }

    public FeedForwardController(double kS, double kV){
        this(kS, kV, 0);
    }

    public ProfileGains.FeedForwardGains getGains(){
        return gains;
    }

    public void setGains(ProfileGains.FeedForwardGains gains){
        this.gains = gains;
    }

    public Output calculate(double velocity, double acceleration){
        return ()-> Math.signum(velocity) * gains.kS() + gains.kV() * velocity + gains.kA() * acceleration;
    }

}
