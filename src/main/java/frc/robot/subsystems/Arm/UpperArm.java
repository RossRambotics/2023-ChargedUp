// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.RobotContainer;

import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** A robot arm subsystem that moves with a motion profile. */
public class UpperArm extends ProfiledPIDSubsystem {
    private TrapezoidProfile.State m_lastSetPoint = new TrapezoidProfile.State(0, 0);
    private GenericEntry m_nt_angle_goal;
    private GenericEntry m_nt_angle_goal_test;
    private GenericEntry m_nt_angle_actual;
    private GenericEntry m_nt_volts;
    private GenericEntry m_nt_feed_forward;

    private Boolean m_testMode = false;

    private final CANSparkMax m_motor = new CANSparkMax(Constants.kMotorPort, MotorType.kBrushless);
    private final WPI_CANCoder m_encoder = new WPI_CANCoder(Constants.kEncoderPort);
    // private final RelativeEncoder m_encoder = m_motor.getEncoder();

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            Constants.kSVolts, Constants.kGVolts,
            Constants.kVVoltSecondPerRad, Constants.kAVoltSecondSquaredPerRad);

    /** Create a new ArmSubsystem. */
    public UpperArm() {
        super(
                new ProfiledPIDController(
                        Constants.kP,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                Constants.kMaxVelocityRadPerSecond,
                                Constants.kMaxAccelerationRadPerSecSquared)),
                0);

        // Start arm at rest in neutral position
        setGoal(Constants.kArmOffsetRads);

        // do all of the CANCoder initialization stuff
        CANCoderConfiguration config = new CANCoderConfiguration();

        // set units of the CANCoder to radians, with velocity being radians per second
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorDirection = true;
        m_encoder.configAllSettings(config);

        // m_encoder.setPositionConversionFactor((2 * Math.PI) / 277); // change the
        // encoder to radians
        m_motor.setInverted(true);
        // m_encoder.setPosition(Math.toRadians(-110.0));

        DataLogManager.log("Upper Arm Position: " + m_encoder.getPosition()); // prints the position of the
                                                                              // CANCoder
        DataLogManager.log("Upper Arm absolute Position: " + m_encoder.getPosition());
        DataLogManager.log("Upper Arm absolute Position: " + m_encoder.getAbsolutePosition());

        // ErrorCode error = m_encoder.getLastError(); // gets the last error generated
        // by the CANCoder
        CANCoderFaults faults = new CANCoderFaults();
        ErrorCode faultsError = m_encoder.getFaults(faults); // fills faults with the
        // current CANCoder faults; returns the
        // last error generated

        m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10); //
        // changes the period of the sensor data frame
        // to 10ms

        // do all of the SparkMax initialization stuff

        ProfiledPIDController p = this.getController();
        p.setIntegratorRange(-0.25, 0.25);
        p.setI(Constants.kI);

    }

    @Override
    public void enable() {
        // do super enable
        // TrapezoidProfile.State temp = m_lastSetPoint;
        super.enable();

        // now do redo reset with the last set point
        // super.getController().reset(m_lastSetPoint);
        // DataLogManager.log(
        // "Upper Arm enable: last pos: " + temp.position + " last vel: " +
        // temp.velocity);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // save last setpoint to blend set points together
        m_lastSetPoint.velocity = setpoint.velocity;
        m_lastSetPoint.position = setpoint.position;

        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // feedforward = 0;
        // Add the feedforward to the PID output to get the motor output

        double volts = MathUtil.clamp(output + feedforward, -10.0, 10.0);
        m_motor.setVoltage(volts);
        m_nt_volts.setDouble(volts);
        m_nt_feed_forward.setDouble(feedforward);

        // DataLogManager.log("Upper Arm volts: " + volts + " output: " + output + " FF:
        // " + feedforward + " Measurement: "
        // + getMeasurement() + " Goal: "
        // + this.m_controller.getGoal().position);
    }

    @Override
    public double getMeasurement() {
        return m_encoder.getPosition() + Constants.kArmOffsetRads;
    }

    public final class Constants {
        public static final int kMotorPort = 60;

        public static final double kP = 8;
        public static final double kI = 0.1;

        public static final double kSVolts = 0.05;
        public static final double kGVolts = 1.0;
        public static final double kVVoltSecondPerRad = 6.5;
        public static final double kAVoltSecondSquaredPerRad = 0.1;

        public static final double kMaxVelocityRadPerSecond = 5;
        public static final double kMaxAccelerationRadPerSecSquared = 3;

        public static final int kEncoderPort = 32;

        // The offset of the arm from the horizontal in its neutral position,
        // measured from the horizontal
        public static final double kArmOffsetRads = -4.270 - (0.0175 * (15 - 3.8));
    }

    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Upper Arm Goal", this.m_controller.getGoal().position);
        SmartDashboard.putNumber("Upper Arm Encoder", m_encoder.getPosition());
        m_nt_angle_goal.setDouble((Math.toDegrees(this.m_controller.getGoal().position)));
        m_nt_angle_actual.setDouble(Math.toDegrees(m_encoder.getPosition() + Constants.kArmOffsetRads));

        if (m_testMode) {
            double goal = m_nt_angle_goal_test.getDouble(0.0);
            goal = Math.toRadians(goal);
            this.setGoal(goal);
            System.out.println("Upper Arm Test Radians: " + goal);
            m_testMode = false;
            this.enable();
        }

    }

    public void createShuffleBoardTab() {
        m_nt_angle_goal_test = RobotContainer.m_armTab.add("U-Arm Test deg", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(0, 0).withProperties(Map.of("min", -160, "max", 160)).getEntry();

        m_nt_angle_goal = RobotContainer.m_armTab.add("U Arm Goal deg", 0)
                .withSize(1, 1)
                .withPosition(2, 1).getEntry();

        m_nt_angle_actual = RobotContainer.m_armTab.add("U Arm Actual deg", 0)
                .withSize(1, 1)
                .withPosition(0, 1).getEntry();

        m_nt_volts = RobotContainer.m_armTab.add("U Arm Volts", 0)
                .withSize(1, 1)
                .withPosition(1, 1).getEntry();

        m_nt_feed_forward = RobotContainer.m_armTab.add("U Arm FF V", 0)
                .withSize(1, 1)
                .withPosition(1, 2).getEntry();

        ShuffleboardLayout pidLayout = RobotContainer.m_armTab.getLayout("Upper Arm", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withPosition(0, 3)
                .withProperties(Map.of("Label position", "HIDDEN"));

        pidLayout.add(this.getController());

        CommandBase cmd = Commands.runOnce(
                () -> m_testMode = true,
                this);
        cmd.setName("U Arm Test");
        RobotContainer.m_armTab.add(cmd)
                .withSize(1, 1)
                .withPosition(3, 1)
                .withProperties(Map.of("Label position", "HIDDEN"));

        cmd = new FunctionalCommand(() -> this.kVTestStart(2.0), //
                () -> {
                }, //
                (b) -> this.kVTestStop(), //
                () -> {
                    return false;
                }) //
                .withTimeout(0.5);
        cmd.setName("U kV Test 2V, 0.5s");

        RobotContainer.m_armTab.add(cmd)
                .withSize(1, 1)
                .withPosition(3, 2);

        cmd = new FunctionalCommand(() -> this.kVTestStart(3.0), //
                () -> {
                }, //
                (b) -> this.kVTestStop(), //
                () -> {
                    return false;
                }) //
                .withTimeout(0.5);
        cmd.setName("U kV Test 3V, 0.5s");

        RobotContainer.m_armTab.add(cmd)
                .withSize(1, 1)
                .withPosition(3, 3);

        cmd = new FunctionalCommand(() -> this.kVTestStart(4.0), //
                () -> {
                }, //
                (b) -> this.kVTestStop(), //
                () -> {
                    return false;
                }) //
                .withTimeout(0.5);
        cmd.setName("U kV Test 4V, 0.5s");
        RobotContainer.m_armTab.add(cmd)
                .withSize(1, 1)
                .withPosition(3, 4);
    }

    private Timer m_testTimer = new Timer();
    private double m_testStartRad = 0.0;
    private double m_testVolts = 0.0;

    public void kVTestStart(double volts) {
        this.disable();
        m_testVolts = volts;

        m_testStartRad = m_encoder.getPosition();
        m_testTimer.reset();
        m_testTimer.start();
        m_motor.setVoltage(m_testVolts);
    }

    public void kVTestStop() {

        double testEndRad = m_encoder.getPosition();
        m_motor.setVoltage(0);
        m_testTimer.stop();

        double kV = (m_testTimer.get() * m_testVolts) / Math.abs(m_testStartRad - testEndRad);

        System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        System.out.println("Start Rad: " + m_testStartRad);
        System.out.println("End Rad: " + testEndRad);
        System.out.println("Time: " + m_testTimer.get());
        System.out.println("Volts: " + m_testVolts);
        System.out.println("kV: " + kV);
        System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
}
