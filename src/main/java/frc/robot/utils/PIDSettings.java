package frc.robot.utils;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;

/** A record that contains the proportional, integral, and derivative gains used by a PID controller. */
public record PIDSettings(double p, double i, double d) { 
    /** Create a PIDController with the gains specified by this PIDSettings object */
    public PIDController toController() {
        return new PIDController(p, i, d);
    }

    /** Copy the gains of this record to the given PID controller.
     * @param other the PID controller to copy the gains to.
     */
    public void copyTo(PIDController other) {
        other.setPID(p, i, d);
    }
}
