package frc.robot.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** A class representing the shooter on the arm */
public class Shooter {
    public SparkMax leftMotor;
    public SparkMax rightMotor;

    /** Create a new Shooter with the given left and right motors.
     * @param leftMotorPort The left motor port of the shooter.
     * @param rightMotorPort The right motor port of the shooter.
     */
    public Shooter(int leftMotorPort, int rightMotorPort) {
        leftMotor = new SparkMax(leftMotorPort, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorPort, MotorType.kBrushless);

        SparkMaxConfig c = new SparkMaxConfig();
        c.idleMode(IdleMode.kBrake);

        leftMotor.configure(c, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(c, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** Set the speed of the shooter's motors.
     * @param speed The speed to set the motors to.
     */
    public void setSpeed(double speed) {
        speed *= 0.75;
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
}
