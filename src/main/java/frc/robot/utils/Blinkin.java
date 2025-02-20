package frc.robot.utils;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {
    private static Spark controller = new Spark(2);

    public static void setLight(double amount) {
        controller.set(amount);
    }
}
