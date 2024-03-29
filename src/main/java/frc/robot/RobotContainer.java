package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.LowerArmSetPoint;
import frc.robot.commands.Drive.SnapDrive;
import frc.robot.commands.Drive.SnapDriveGamePiece;
import frc.robot.commands.Drive.SnapDriveToPortal;
import frc.robot.commands.Drive.SnapDriveToPoseField;
import frc.robot.commands.Grabber.AutoGrab;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.IntakeOn;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Positioning.TrackingButton;
import frc.robot.commands.Positioning.VisionOff;
import frc.robot.commands.Positioning.VisionOn;
import frc.robot.commands.Tracking.EnableLight;
import frc.robot.commands.auto.AutoBlueFive;
import frc.robot.commands.auto.AutoBlueHighNothing;
import frc.robot.commands.auto.AutoBlueNine;
import frc.robot.commands.auto.AutoBlueOne;
import frc.robot.commands.auto.AutoBlueTwo;
import frc.robot.commands.auto.AutoMoveBackToPose;
import frc.robot.commands.auto.AutoMoveConeLeft;
import frc.robot.commands.auto.AutoRedFive;
import frc.robot.commands.auto.AutoRedHighNothing;
import frc.robot.commands.auto.AutoRedNine;
import frc.robot.commands.auto.AutoRedOne;
import frc.robot.commands.auto.Blue2DriveUpChargeStation;
import frc.robot.commands.auto.BlueDriveUpChargeStation;
import frc.robot.commands.auto.CalibrateRedX;
import frc.robot.commands.auto.CalibrateRedXY;
import frc.robot.commands.auto.RedDriveUpChargeStation;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.DrivetrainSubsystem;

import frc.robot.subsystems.GridSelector;

import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Positioning;

import frc.robot.subsystems.Tracking;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.LowerArm;
import frc.robot.subsystems.Arm.UpperArm;
import frc.robot.subsystems.Intake.IntakeFrame;
import frc.robot.subsystems.Intake.IntakeWheels;
import frc.robot.subsystems.LEDs.LEDPanel;

public class RobotContainer {

    private static RobotContainer m_theRobot = null;

    public static void setTheRobot(RobotContainer r) {
        m_theRobot = r;
    }

    public static RobotContainer getTheRobot() {
        return m_theRobot;
    }

    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Match.Auto");
    static public final ShuffleboardTab m_TuningTab = Shuffleboard.getTab("Match.Tuning");
    static public final ShuffleboardTab m_armTab = Shuffleboard.getTab("Arm");
    static public final ShuffleboardTab m_buttonBoxTab = Shuffleboard.getTab("Button.Box");

    static public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    static public final GridSelector m_GridSelector = new GridSelector();

    static public final Positioning m_positioning = new Positioning();
    static public final Arm m_arm = new Arm();
    static public final UpperArm m_upperArm = new UpperArm();
    static public final LowerArm m_lowerArm = new LowerArm();
    static public final Grabber m_grabber = new Grabber();
    static public final IntakeFrame m_intakeFrame = new IntakeFrame();
    static public final IntakeWheels m_intakeWheels = new IntakeWheels();

    static public final Tracking m_Tracking = new Tracking();
    private static double slewLimit = 0.6;

    // LED subsystem
    static public final LEDPanel m_LEDPanel = new LEDPanel();

    private final XboxController m_controllerDriver = new XboxController(0);
    private Joystick m_gridSelector2 = new Joystick(2);
    private Joystick m_gridSelector = new Joystick(1);
    private Joystick m_intakeController = new Joystick(3);
    // private final XboxController m_controllerOperator = new XboxController(1);
    Trigger xButton = new JoystickButton(m_controllerDriver, XboxController.Button.kX.value);
    Trigger leftBumper = new JoystickButton(m_controllerDriver, XboxController.Button.kLeftBumper.value);
    Trigger rightBumper = new JoystickButton(m_controllerDriver, XboxController.Button.kRightBumper.value);
    Trigger backButton = new JoystickButton(m_controllerDriver, XboxController.Button.kBack.value);
    Trigger startButton = new JoystickButton(m_controllerDriver, XboxController.Button.kStart.value);
    Trigger aButton = new JoystickButton(m_controllerDriver, XboxController.Button.kA.value);
    Trigger bButton = new JoystickButton(m_controllerDriver, XboxController.Button.kB.value);
    Trigger yButton = new JoystickButton(m_controllerDriver, XboxController.Button.kY.value);
    Trigger btnCloseJaws = new JoystickButton(m_gridSelector2, 7);
    Trigger btnOpenJaws = new JoystickButton(m_gridSelector, 11);
    Trigger btnResetVision = new JoystickButton(m_gridSelector, 12);
    Trigger btnHuman = new JoystickButton(m_gridSelector2, 1);
    Trigger btn3rdFloor = new JoystickButton(m_gridSelector2, 10);
    Trigger btn2ndFloor = new JoystickButton(m_gridSelector2, 11);
    Trigger btn1stFloor = new JoystickButton(m_gridSelector2, 12);
    Trigger rightTrigger = new Trigger(
            () -> m_controllerDriver.getRawAxis(XboxController.Axis.kRightTrigger.value) >= 0.5);
    Trigger leftTrigger = new Trigger(
            () -> m_controllerDriver.getRawAxis(XboxController.Axis.kLeftTrigger.value) >= 0.5);
    Trigger intakeDown = new Trigger(
            () -> m_intakeController.getRawAxis(4) >= 0.5);
    Trigger intakeUp = new Trigger(
            () -> m_intakeController.getRawAxis(4) <= -0.5);

    Trigger intakeOn = new JoystickButton(m_intakeController, 2);
    Trigger intakeReverse = new JoystickButton(m_intakeController, 3);
    public PhysicsSim m_PhysicsSim;

    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation

        m_drivetrainSubsystem.setDefaultCommand(new SnapDrive(m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                () -> snapAngle()));

        // Configure the button bindings
        configureButtonBindings();

        // disable Live Window per recommendations by WPILIB team to reduce network
        // overhead
        // remove this line if stuff is missing from shuffleboard that we need.
        // LiveWindow.disableAllTelemetry();
        // LiveWindow.enableAllTelemetry();
    }

    private SlewRateLimiter m_slewLeftY = new SlewRateLimiter(1.5);

    public double getInputLeftY() {
        double driverLeftY = modifyAxis(m_controllerDriver.getLeftY());

        double slew = m_slewLeftY.calculate(driverLeftY * slewLimit)
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;

        if (m_GridSelector.isBlueAlliance()) {
            return slew;
        } else {
            return -slew;
        }

    }

    private SlewRateLimiter m_slewLeftX = new SlewRateLimiter(1.5);

    public double getInputLeftX() {
        double driverLeftX = modifyAxis(m_controllerDriver.getLeftX());

        double slew = m_slewLeftX.calculate(driverLeftX * slewLimit)
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;

        if (m_GridSelector.isBlueAlliance()) {
            return slew;
        } else {
            return -slew;
        }
    }

    private double getNudgeValue() {
        if (m_GridSelector.isBlueAlliance()) {
            return -0.4;
        } else {
            return 0.4;
        }
    }

    private double getNudgeAngle() {
        if (m_GridSelector.isBlueAlliance()) {
            return 10;
        } else {
            return 10;
        }
    }
    // private double getOperatorRightY() {
    // double operatorRightY = 0;

    // // implement Joystick Deadzone
    // if (Math.abs(m_controllerOperator.getRightY()) > 0.08) {
    // operatorRightY = m_controllerOperator.getRightY();

    // }

    // return operatorRightY;
    // }

    private double m_lastSnapAngle = 720; // defaults to 720 because 720 tells snap drive to not adjust the angle

    public void resetLastSnapAngle() {
        m_lastSnapAngle = 720;
    }

    private double snapAngle() {
        double x, y;

        if (m_GridSelector.isBlueAlliance()) {
            x = -m_controllerDriver.getRightX();
            y = -m_controllerDriver.getRightY();
        } else {
            x = m_controllerDriver.getRightX();
            y = m_controllerDriver.getRightY();
        }

        if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
            // return 720;
            // stop angle drift
            return m_lastSnapAngle;
        }

        double angle = Math.toDegrees(Math.atan2(x, y));

        if ((angle > 360) || (angle < -360)) {
            angle = 360;
        }

        m_lastSnapAngle = angle;
        return angle; // pizza
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Command cmd;

        backButton.whileTrue(new RunCommand(() -> m_drivetrainSubsystem.zeroGyroscope()));

        startButton.onTrue(Commands.runOnce(() -> {
            // m_positioning.resetVision();
            m_drivetrainSubsystem.resetSteerEncoders();
        }));

        btnOpenJaws.whileTrue(new RunCommand(() -> m_grabber.openJaws()));

        btnCloseJaws.whileTrue(new RunCommand(() -> m_grabber.closeJaws()));

        aButton.onTrue(Commands.runOnce(() -> m_grabber.openJaws()));

        bButton.onTrue(Commands.runOnce(() -> m_grabber.closeJaws()));

        yButton.onTrue(Commands.runOnce(() -> m_arm.goNextNode()));

        leftTrigger.onTrue(Commands.runOnce(() -> slewLimit = 1.0));
        leftTrigger.onFalse(Commands.runOnce(() -> slewLimit = 0.6));

        intakeDown.whileTrue(new ExtendIntake());
        intakeUp.whileTrue(new RetractIntake());

        intakeOn.whileTrue(new IntakeOn());
        intakeReverse.whileTrue(new IntakeReverse());

        // cmd = new DefaultDriveCommand(
        // m_drivetrainSubsystem,
        // () -> -getInputLeftY(),
        // () -> -getInputLeftX(),
        // () -> {
        // return -0.2;
        // });
        // new POVButton(m_controllerDriver, 180)
        // .whenHeld(cmd);

        // cmd = new DefaultDriveCommand(
        // m_drivetrainSubsystem,
        // () -> -getInputLeftY(),
        // () -> -getInputLeftX(),
        // () -> {
        // return 0.2;
        // });
        // new POVButton(m_controllerDriver, 90)
        // .whenHeld(cmd);

        /**
         * Operator controls
         */

        /**
         * Implement Snap Drive
         */

        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> -getNudgeValue(),
                () -> 0,
                () -> RobotContainer.m_drivetrainSubsystem.getGyroHeading().getDegrees());

        new POVButton(m_controllerDriver, 0).whileTrue(cmd);

        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> getNudgeValue(),
                () -> 0,
                () -> RobotContainer.m_drivetrainSubsystem.getGyroHeading().getDegrees());

        new POVButton(m_controllerDriver, 180).whileTrue(cmd);

        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> 0,
                () -> getNudgeValue(),
                () -> RobotContainer.m_drivetrainSubsystem.getGyroHeading().getDegrees());

        new POVButton(m_controllerDriver, 90).whileTrue(cmd);

        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> 0,
                () -> -getNudgeValue(),
                () -> RobotContainer.m_drivetrainSubsystem.getGyroHeading().getDegrees());

        new POVButton(m_controllerDriver, 270).whileTrue(cmd);

        // Turn Left with Left Bumper
        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                () -> RobotContainer.m_drivetrainSubsystem.getGyroHeading().getDegrees() + getNudgeAngle());

        leftBumper.whileTrue(cmd);

        // Turn Right with Right Bumper
        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                () -> RobotContainer.m_drivetrainSubsystem.getGyroHeading().getDegrees() - getNudgeAngle());

        rightBumper.whileTrue(cmd);

        // new POVButton(m_controllerOperator, 180)
        // .whenHeld(cmd);

        // East
        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                90);

        // new POVButton(m_controllerOperator, 90)
        // .whenHeld(cmd);

        // West
        cmd = new frc.robot.commands.Drive.SnapDrive(
                m_drivetrainSubsystem,
                () -> -getInputLeftY(),
                () -> -getInputLeftX(),
                270);

        // new POVButton(m_controllerOperator, 270)
        // .whenHeld(cmd);

        // map button for tracking cargo
        // create tracking cargo drive command

        // cmd = new ParallelDeadlineGroup(new AutoGrab(),
        // new ParallelCommandGroup(
        // new InstantCommand(() -> RobotContainer.m_grabber.openJaws()),
        // new SnapDriveGamePiece(m_drivetrainSubsystem,
        // () -> -getInputLeftY(),
        // () -> -getInputLeftX(),
        // () -> m_Tracking.getTargetHeading()),
        // new EnableLight()));

        cmd = new TrackingButton();
        cmd.setName("TrackingButton");
        xButton.whileTrue(cmd);

        cmd = new SnapDriveToPortal(
                RobotContainer.m_drivetrainSubsystem,
                0.05);
        cmd.setName("SnapDriveToPortal");
        rightTrigger.whileTrue(cmd);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        CommandBase cmd = (CommandBase) m_autoChooser.getSelected();
        if (cmd == null) {
            return new InstantCommand();
        }
        return cmd;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.1);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    private SendableChooser m_autoChooser = new SendableChooser();

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout commands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 1)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        CommandBase cmd;

        // Add auto routines
        CommandBase autoCmd = null;
        autoCmd = new InstantCommand();
        autoCmd.setName("Do Nothing");
        m_autoChooser.addOption("Do Nothing", autoCmd);

        autoCmd = new BlueDriveUpChargeStation();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new RedDriveUpChargeStation();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new AutoRedNine();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new AutoBlueOne();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new AutoBlueTwo();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new AutoBlueNine();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new AutoRedOne();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new AutoRedFive();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new AutoBlueFive();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new AutoRedHighNothing();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new AutoBlueHighNothing();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new Blue2DriveUpChargeStation();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new CalibrateRedX();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new CalibrateRedXY();
        m_autoChooser.addOption(autoCmd.getName(), autoCmd);
        commands.add(autoCmd);

        autoCmd = new ExtendIntake();
        commands.add(autoCmd);

        autoCmd = new RetractIntake();
        commands.add(autoCmd);

        autoCmd = new IntakeOn();
        commands.add(autoCmd);

        autoCmd = new IntakeReverse();
        commands.add(autoCmd);

        autoCmd = new VisionOff();
        commands.add(autoCmd);

        autoCmd = new VisionOn();
        commands.add(autoCmd);

        tab.add("Autonomous", m_autoChooser).withSize(2, 1);

        this.m_Tracking.createShuffleBoardTab();
        this.m_lowerArm.createShuffleBoardTab();
        this.m_upperArm.createShuffleBoardTab();
        this.m_arm.createShuffleBoardTab();

        DataLogManager.start();
        DataLogManager.log("Log Started.");
    }
}