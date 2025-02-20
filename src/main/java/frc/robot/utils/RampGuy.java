package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RampGuy {
    double lastInput = 0;
    double maxRamp;
    boolean gotToSpeed = false;

    public RampGuy(double maxRamp) {this.maxRamp = maxRamp;}

    public void reset() {
        lastInput = 0;
        gotToSpeed = false;
    }

    public double calculate(double target) {
        double delta = target - lastInput;
        if (MathUtil.isNear(delta, 0.0, 0.05) && !MathUtil.isNear(target, 0.0, 0.1)) {
            gotToSpeed = true;
        }

        SmartDashboard.putNumber("ramp-target", target);
        SmartDashboard.putNumber("ramp-delta", delta);
        SmartDashboard.putNumber("at-speed", gotToSpeed ? 1 : 0);
        if (gotToSpeed) {
            SmartDashboard.putNumber("ramp-output", target);
            return target;
        }

        double output = 0;

        if (target > 0) {
            double newTarg = lastInput + maxRamp;
            output = Math.min(newTarg, target);
        } else {
            double newTarg = lastInput - maxRamp;
            output = Math.max(newTarg, target);
        }

        SmartDashboard.putNumber("ramp-output", output);

        lastInput = output;
        return output;
    }
}
