package frc.robot.tracking;

import frc.robot.Constants.ArmConstants;

/** A class that is used to manage tracking */
public class Tracking {
    private static Tracking instance;

    private TrackingState state = TrackingState.None;
    
    /** Get the current instance of the tracking system, or create it if it does not exist yet.
     * @return The current intance of the tracking system.
     */
    public static Tracking get() {
        if (instance == null) {
            instance = new Tracking();
        }
        return instance;
    }

    /** Set the current state of the tracking system.
     * @param state The state to set the tracking system to.
     */
    public void setState(TrackingState state) {
        this.state = state;
    }

    public static boolean enabled() {
        return get().state != TrackingState.None;
    }

    public static boolean disabled() {
        return get().state == TrackingState.None;
    }

    /** Get the current state of the tracking system.
     * @return The current state of the tracking system.
     */
    public TrackingState getState() {
        return state;
    }

    /** Get the calculated angle to hold the arm at, or 0.0 if the tracking system is disabled.
     * @return The angle to hold the arm at.
     */
    public double getArmAngle() {
        if (state == TrackingState.Amp) {
            return ArmConstants.armAmp;
        }
        if (state == TrackingState.Speaker) {
            System.out.println("[ERROR] Speaker tracking not implemented");
        }
        return 0.0;
    }

    /** Get the calculated drivetrain rotation speed, or 0.0 if the tracking system is disabled.
     * @return The drivetrain rotation speed.
     */
    public double getRobotRotationSpeed() {
        return 0.0;
        //if (state == TrackingState.Amp) {
            //return Limelight.getTagFieldPos().yaw() / 15.0;
        //}
        //if (state == TrackingState.Speaker) {
            //System.out.println("[ERROR] Speaker tracking not implemented");
        //}
        //return 0.0;
    }

    /** An enum that represents a target for the tracking system. */
    public static enum TrackingState {
        None,
        Amp,
        Speaker,
    }
}
