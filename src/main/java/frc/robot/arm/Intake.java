package frc.robot.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** A class representing the intake on the arm */
public class Intake {
    private SparkMax motor;

    /** Create a new Intake with the given motor port.
     * @param motorPort The port of the intake motor.
     */
    public Intake(int motorPort) {
        motor = new SparkMax(motorPort, MotorType.kBrushless);
    }

    /** Set the speed of the intake's motor.
     * @param speed The speed to set the motor to.
     */
    public void setSpeed(double speed) {
        speed *= 0.8;
        motor.set(speed);
    }
}
