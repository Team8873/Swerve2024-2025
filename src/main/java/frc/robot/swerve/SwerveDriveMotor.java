package frc.robot.swerve;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.utils.PIDSettings;

/** A class representing a motor in a swerve drive module */
public class SwerveDriveMotor {
    private SparkMax motor;
    private SparkClosedLoopController velocityPid;
    private RelativeEncoder encoder;
    
    private double targetVelocity;
    private double currentSpeed;

    /** Create a new SwerveDriveMotor on the given port with the given PID settings.
     * @param motorPort The port of the swerve motor.
     * @param pid The PID settings to use.
     */
    public SwerveDriveMotor(int motorPort, PIDSettings pid) {
        motor = new SparkMax(motorPort, MotorType.kBrushless);
        SparkMaxConfig c = new SparkMaxConfig();
        c.encoder.positionConversionFactor(Constants.SwerveConstants.driveEncoderScaleFactor).velocityConversionFactor(SwerveConstants.driveEncoderVelocityFactor);
        c.smartCurrentLimit(40).idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        // c.closedLoop.pidf(0.5, 0.001, 0.05, 0.25).outputRange(-1.0, 1.0).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        motor.configure(c, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        targetVelocity = 0;
        currentSpeed = 0;
        encoder = motor.getEncoder();
        velocityPid = motor.getClosedLoopController();

        UIConstants.debug.addDouble(motorPort + " target velocity", () -> targetVelocity);
        UIConstants.debug.addDouble(motorPort + " current velocity", () -> encoder.getVelocity());
        UIConstants.debug.addDouble(motorPort + " position", () -> encoder.getPosition());
        UIConstants.debug.addDouble(motorPort + " drive power", () -> motor.getAppliedOutput());
        UIConstants.current.addDouble(motorPort + " output current", () -> motor.getOutputCurrent());
        UIConstants.current.addDouble(motorPort + " bus voltage", () -> motor.getBusVoltage());
    }

    public void setDriveMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode mode) {
        SparkMaxConfig c = new SparkMaxConfig();
        c.idleMode(mode);
        motor.configure(c, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** Set the target velocity of the motor.
     * @param velocity The target velocity, in meters per second.
     */
    double prevError = 0.0;
    public void setTarget(double velocity) {
        motor.set(velocity / 20);
    }

    /** Get the position of the motor's encoder.
     * @return The position of the motor's encoder.
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /** Get the velocity of the motor.
     * @return The velocity of the motor.
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /** Initialize the motor's UI on Shuffleboard.
     * @param name The name of the module this motor is attached to.
     * @param column The column to place UI elements on.
     */
    public void setupUI(String name, int column) {
    }
}
