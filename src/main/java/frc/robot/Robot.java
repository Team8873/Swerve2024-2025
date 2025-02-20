// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.tracking.Tracking.TrackingState;
import frc.robot.tracking.Limelight;
import frc.robot.tracking.Tracking;
import frc.robot.arm.Arm;
import frc.robot.auto.AutoState;
import frc.robot.climber.Climber;
import frc.robot.input.InputPacket;
import frc.robot.swerve.SwerveDrivetrain;
import frc.robot.ui.Position;
import frc.robot.ui.SimpleButton;
import frc.robot.utils.Blinkin;
import frc.robot.utils.DistanceSensor;
import frc.robot.utils.ParameterStore;
import frc.robot.utils.TaskRunner;
import frc.robot.auto.AutoTask;

public class Robot extends TimedRobot {
    private final AHRS gyroscope = new AHRS();
    private final XboxController driveController = new XboxController(DriveConstants.controllerPort);
    private final XboxController operatorController = new XboxController(DriveConstants.operatorPort);
    private final Arm arm = new Arm();
    private final Climber climber = new Climber();

    private SendableChooser<String> autoChooser;
    private static String fourNote = "(Center) Four Note";
    private static String twoNote = "(Center) Two Note";
    private static String threeNoteL = "(Center) Three Note, Left Bias";
    private static String threeNoteR = "(Center) Three Note, Right Bias";
    private static String leftAndLeave = "(Left) Drive and Score";
    private static String rightAndLeave = "(Right) Drive and Score";
    private static String shootOnly = "Score Only";
    private static String finalsBlue = "(BLUE) Finals Blue Left";
    private static String finalsRed = "(RED) Final Red Right";
    private static String noAuto = "Do Nothing";

    private TaskRunner<AutoState> autoTaskRunner;

    @Override
    public void robotInit() {
        SwerveDrivetrain.init(gyroscope);

        SimpleButton.createButton(UIConstants.tuning, "Save", new Position(0,4), ParameterStore::saveStore);

        Limelight.camStreamSetup();
        DistanceSensor.init();
        autoState = new AutoState(() -> arm.getArmAngle(), () -> arm.getShooterSpeed());


        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption(fourNote, fourNote);
        autoChooser.addOption(threeNoteL, threeNoteL);
        autoChooser.addOption(threeNoteR, threeNoteR);
        autoChooser.addOption(twoNote, twoNote);
        autoChooser.addOption(leftAndLeave, leftAndLeave);
        autoChooser.addOption(rightAndLeave, rightAndLeave);
        autoChooser.addOption(finalsBlue, finalsBlue);
        autoChooser.addOption(finalsRed, finalsRed);
        autoChooser.addOption(shootOnly, shootOnly);
        autoChooser.addOption(noAuto, noAuto);

        SmartDashboard.putData("auto", autoChooser);

        autoTaskRunner = new TaskRunner<>();

        autoTaskRunner.withDefault((s) -> {
            s.done = true;
        }).onStateChange((s) -> {
            SwerveDrivetrain.resetTimer();
        });
    }

    @Override
    public void robotPeriodic() {
        SimpleButton.updateAll();
        SwerveDrivetrain.updateEncoders();
        arm.updateEncoders();
        SwerveDrivetrain.updateOdometry();

        SmartDashboard.putNumber("dist", DistanceSensor.distance());
        SmartDashboard.putBoolean("auto-done", autoState.done);

        if (arm.isNoteInIntake()) {
            Blinkin.setLight(-0.89);
        } else {
            var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            Blinkin.setLight(alliance == Alliance.Blue ? 0.85 : 0.61);
        }
    }

    private AutoState autoState;

    @Override
    public void autonomousInit() {
        SwerveDrivetrain.setDriveMode(IdleMode.kCoast);
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            Limelight.setPriority(7);
        } else {
            Limelight.setPriority(4);
        }
        AutoConstants.calculateNotePositions();
        Limelight.camModeVision();
        SwerveDrivetrain.setPosition(Limelight.getRobotPos());
        arm.onModeInit();
        autoState = new AutoState(() -> arm.getArmAngle(), () -> arm.getShooterSpeed());
        SwerveDrivetrain.resetTimer();
        autoTaskRunner.clear();
        var selected = autoChooser.getSelected();

        if (selected == noAuto) {
            return;
        } else if (selected == fourNote) {
            queueFourNote();
        } else if (selected == threeNoteL) {
            queueThreeNoteLeft();
        } else if (selected == threeNoteR) {
            queueThreeNoteRight();
        } else if (selected == twoNote) {
            queueTwoNote();
        } else if (selected == shootOnly) {
            autoTaskRunner.then(AutoTask.prepShoot()).then(AutoTask.shoot());
        } else if (selected == finalsBlue) {
            autoTaskRunner.then(new AutoTask().toDurationTask(50 * 10));
            queueLeft(Alliance.Blue);
        } else if (selected == finalsRed) {
            autoTaskRunner.then(new AutoTask().toDurationTask(50 * 10));
            queueRight(Alliance.Red);
        } else if (selected == leftAndLeave) {
            var alliance = DriverStation.getAlliance();
            if (alliance.isEmpty()) {
                System.out.println("[WARNING]: No alliance detected, auton will not start.");
                return;
            }
            queueLeft(alliance.get());
        } else if (selected == rightAndLeave) {
            var alliance = DriverStation.getAlliance();
            if (alliance.isEmpty()) {
                System.out.println("[WARNING]: No alliance detected, auton will not start.");
                return;
            }
            queueRight(alliance.get());
        }
        return;
    }

    private void queueFourNote() {
        autoTaskRunner
            .then(AutoTask.prepShoot())
            .then(AutoTask.shoot())

            .then(AutoTask.toNote1H())
            .then(AutoTask.toNote1V())
            .then(AutoTask.toSpeaker())
            .then(AutoTask.shoot())

            .then(AutoTask.toNote2())
            .then(AutoTask.toSpeaker())
            .then(AutoTask.shoot())

            .then(AutoTask.toNote3H())
            .then(AutoTask.toNote3V())
            .then(AutoTask.toSpeaker())
            .then(AutoTask.shoot());
    }

    private void queueThreeNoteLeft() {
        autoTaskRunner
            .then(AutoTask.prepShoot())
            .then(AutoTask.shoot())

            .then(AutoTask.toNote2())
            .then(AutoTask.toSpeaker())
            .then(AutoTask.shoot())

            .then(AutoTask.toNote1H())
            .then(AutoTask.toNote1V())
            .then(AutoTask.toSpeaker())
            .then(AutoTask.shoot());
    }

    private void queueThreeNoteRight() {
        autoTaskRunner
            .then(AutoTask.prepShoot())
            .then(AutoTask.shoot())

            .then(AutoTask.toNote2())
            .then(AutoTask.toSpeaker())
            .then(AutoTask.shoot())

            .then(AutoTask.toNote3H())
            .then(AutoTask.toNote3V())
            .then(AutoTask.toSpeaker())
            .then(AutoTask.shoot());
    }

    private void queueTwoNote() {
        autoTaskRunner
            .then(AutoTask.prepShoot())
            .then(AutoTask.shoot())

            .then(AutoTask.toNote2())
            .then(AutoTask.toSpeaker())
            .then(AutoTask.shoot());
    }

    private void queueLeft(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            var targetPos = new Translation2d(AutoConstants.note1VPos.getX() + 1.5, AutoConstants.note1VPos.getY());
            autoTaskRunner
                .then(AutoTask.prepShoot())
                .then(AutoTask.shoot())
                .then(AutoTask.toNote1H())
                .then(new AutoTask().moveTo(targetPos, 0.0).toTask());
        } else {
            var targetPosH = new Translation2d(AutoConstants.note1HPos.getX(), AutoConstants.note1VPos.getY() + 2.0);
            var targetPosV = new Translation2d(AutoConstants.note1VPos.getX() + 1.5, AutoConstants.note1VPos.getY() + 2.0);
            autoTaskRunner
                .then(AutoTask.prepShoot())
                .then(AutoTask.shoot())
                .then(new AutoTask().moveTo(targetPosH, 0.0).toTask())
                .then(new AutoTask().moveTo(targetPosV, 0.0).toTask());
        }
    }

    private void queueRight(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            var targetPosH = new Translation2d(AutoConstants.note3HPos.getX(), AutoConstants.note3VPos.getY() - 2.0);
            var targetPosV = new Translation2d(AutoConstants.note3VPos.getX() + 1.5, AutoConstants.note3VPos.getY() - 2.0);
            autoTaskRunner
                .then(AutoTask.prepShoot())
                .then(AutoTask.shoot())
                .then(new AutoTask().moveTo(targetPosH, 0.0).toTask())
                .then(new AutoTask().moveTo(targetPosV, 0.0).toTask());
        } else {
            var targetPos = new Translation2d(AutoConstants.note3VPos.getX() + 1.5, AutoConstants.note3VPos.getY());
            autoTaskRunner
                .then(AutoTask.prepShoot())
                .then(AutoTask.shoot())
                .then(AutoTask.toNote3H())
                .then(new AutoTask().moveTo(targetPos, 0.0).toTask());
        }
    }

    @Override
    public void autonomousPeriodic() {
        autoTaskRunner.runOnce(autoState);
        if (autoState.done) {
            arm.handleRawInputs(0.0, 0.0, 0.0, true);
            SwerveDrivetrain.driveRaw(0.0, 0.0, 0.0, getPeriod());
            return;
        }
        SmartDashboard.putNumber("targ state", autoState.armRotationTarget);
        arm.setArmTargetAngle(autoState.armRotationTarget);
        arm.handleRawInputs(0.0, autoState.intakeSpeed, autoState.shooterSpeed, true);
        SwerveDrivetrain.setHoldAngle(autoState.targetRotation);
        SwerveDrivetrain.driveTo(autoState.targetPosition, getPeriod());
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        SwerveDrivetrain.setDriveMode(IdleMode.kCoast);
        SwerveDrivetrain.resetHoldAngle();
        SwerveDrivetrain.setDriveMode(IdleMode.kCoast);
        Tracking.get().setState(TrackingState.None);
        arm.onModeInit();
        Limelight.camModeDriver();

        climber.home();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        var inputs = InputPacket.readFromController(driveController, operatorController);
        if (inputs.tracking() != TrackingState.None) Tracking.get().setState(inputs.tracking());
        SwerveDrivetrain.drive(inputs, getPeriod());
        arm.handleInputs(inputs); 
        climber.handleInputs(inputs);

        if (arm.isShooterSpooled()) {
            driveController.setRumble(RumbleType.kBothRumble, 1.0);
        } else {
            driveController.setRumble(RumbleType.kBothRumble, 0.0);
        }
    }

    @Override
    public void disabledInit() {
        SwerveDrivetrain.setDriveMode(IdleMode.kBrake);
    }
    @Override
    public void disabledPeriodic() {}
    @Override
    public void testInit() {}
    @Override
    public void testPeriodic() {}
    @Override
    public void simulationInit() {}
    @Override
    public void simulationPeriodic() {}
}
