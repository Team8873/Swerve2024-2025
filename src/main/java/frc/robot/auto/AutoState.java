package frc.robot.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ArmConstants;
import frc.robot.swerve.SwerveDrivetrain;

/** A class representing the robot state during auton */
public class AutoState {
    public double armRotationTarget;
    public double intakeSpeed;
    public double shooterSpeed;
    public Translation2d targetPosition;
    public double targetRotation;
    public boolean done;

    private DoubleSupplier armRot;
    private DoubleSupplier shooterRevs;

    /** Create a new default auton state. All parameters are inferred exceupt for the arm rotation which needs to be manually provided.
     * @param armRotationSupplier A supplier for the arm rotation in degrees.
     * @param shooterRevSupplier A supplier for the shooter rpm.
     */
    public AutoState(DoubleSupplier armRotationSupplier, DoubleSupplier shooterRevSupplier) {
        armRot = armRotationSupplier;
        shooterRevs = shooterRevSupplier;

        armRotationTarget = armRot.getAsDouble();
        intakeSpeed = 0.0;
        shooterSpeed = 0.0;
        targetPosition = SwerveDrivetrain.getPosition();
        targetRotation = 0.0;
        done = false;
    }

    /** Checks if the robot is at the current swerve target location.
     * @return Whether the robot is at the correct location.
     */
    public boolean atSwerveTarget() {
        return SwerveDrivetrain.at(targetPosition, 0.03);
    }

    /** Checks if the robot arm is at the current rotation target.
     * @return Whether the arm is at the correct angle.
     */
    public boolean atArmTarget() {
        return MathUtil.isNear(armRotationTarget, armRot.getAsDouble(), 1);
    }

    /** Checks if the shooter is spooled up.
     * @return Whether the shooter is spooled up.
     */
    public boolean isShooterSpooled() {
        return shooterRevs.getAsDouble() > ArmConstants.shooterRevSpeed;
    }
}
