// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.util.ArrayBuilders.BooleanBuilder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.Arm;

/** Add your docs here. */
public class GridSelector extends SubsystemBase {
    private Joystick m_gridSelector = new Joystick(1);
    private Joystick m_gridSelector2 = new Joystick(2);
    private long m_dial = 0;
    private boolean m_leftSwitch_isRed = true;
    private boolean m_isCube = true;
    private boolean m_rightSwitch = true;
    private boolean m_buttonBox = true;

    /** Creates a new GridSelector2. */
    public GridSelector() {

    }

    public void initialize() {
        CommandBase cmd;

        Trigger btnHighArm = new JoystickButton(m_gridSelector2, 2);
        // CommandBase cmd = new PrintCommand("************** Button Pressed!
        // *******************");
        cmd = Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.Z);
        btnHighArm.onTrue(cmd);

        Trigger btnCarry = new JoystickButton(m_gridSelector2, 5);
        cmd = Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.A);
        btnCarry.onTrue(cmd);

        Trigger btnThirdFloor = new JoystickButton(m_gridSelector2, 10);
        cmd = Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.B);
        btnThirdFloor.onTrue(cmd);

        Trigger btnTrunk = new JoystickButton(m_gridSelector2, 4);
        cmd = Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.S);
        btnTrunk.onTrue(cmd);

        Trigger btnSecondFloor = new JoystickButton(m_gridSelector2, 11);
        cmd = Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.D);
        btnSecondFloor.onTrue(cmd);

        Trigger btnLowArm = new JoystickButton(m_gridSelector2, 8);
        cmd = Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.O);
        btnLowArm.onTrue(cmd);

        Trigger btnPortal = new JoystickButton(m_gridSelector2, 3);
        cmd = Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.YY);
        btnPortal.onTrue(cmd);

        Trigger btnFirstFloor = new JoystickButton(m_gridSelector2, 12);
        cmd = Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.L);
        btnFirstFloor.onTrue(cmd);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_buttonBox == true) {
            if (m_gridSelector.getRawButton(1)) {
                m_dial = 1;
            } else if (m_gridSelector.getRawButton(2)) {
                m_dial = 2;
            } else if (m_gridSelector.getRawButton(3)) {
                m_dial = 3;
            } else if (m_gridSelector.getRawButton(4)) {
                m_dial = 4;
            } else if (m_gridSelector.getRawButton(5)) {
                m_dial = 5;
            } else if (m_gridSelector.getRawButton(6)) {
                m_dial = 6;
            } else if (m_gridSelector.getRawButton(7)) {
                m_dial = 7;
            } else if (m_gridSelector.getRawButton(8)) {
                m_dial = 8;
            } else if (m_gridSelector.getRawButton(10)) {
                m_dial = 9;
            } else {
                m_dial = 0;
            }

            if (m_gridSelector.getRawAxis(0) > -.5) {
                // Red and Blue switch on Red
                m_leftSwitch_isRed = true;
            } else if (m_gridSelector.getRawAxis(0) < -.5) {
                // Red and Blue switch on Blue
                m_leftSwitch_isRed = false;
            }

            if (m_gridSelector.getRawAxis(1) < -.5) {
                // Cube and Cone switch on Cube
                m_isCube = true;
            } else if (m_gridSelector.getRawAxis(0) > -.5) {
                // Cube and Cone switch on Cone
                m_isCube = false;
            }

            if (m_gridSelector2.getRawAxis(1) > .5) {
                // Tracking is on Grid
                m_rightSwitch = true;
            } else if (m_gridSelector.getRawAxis(0) < .5) {
                // Tracking is on Game Piece
                m_rightSwitch = false;
            }
        } else {
            m_rightSwitch = SmartDashboard.getBoolean("Right Switch", false);
            m_leftSwitch_isRed = SmartDashboard.getBoolean("is Red Alliance", false);
            m_isCube = SmartDashboard.getBoolean("is Cube", false);
        }

        SmartDashboard.putBoolean("is Red Alliance", m_leftSwitch_isRed);
        SmartDashboard.putBoolean("is Cube", m_isCube);
        SmartDashboard.putBoolean("Right Switch", m_rightSwitch);
        SmartDashboard.putBoolean("Button Box", m_buttonBox);

    }

    public Boolean isBlueAlliance() {
        return !m_leftSwitch_isRed;
    }

    public Boolean isCube() {
        return m_isCube;
    }

    public Boolean getRightSwitch() {
        return m_rightSwitch;
    }

    public void toggleLeftSwitch() {
        System.out.println("**********Toggle Left Switch**********");
        if (m_leftSwitch_isRed == true) {
            m_leftSwitch_isRed = false;
        } else if (m_leftSwitch_isRed == false) {
            m_leftSwitch_isRed = true;
        }
    }

    public void toggleMidSwitch() {
        System.out.println("**********Toggle Mid Switch**********");
        if (m_isCube == true) {
            m_isCube = false;
        } else if (m_isCube == false) {
            m_isCube = true;
        }
    }

    public void toggleRightSwitch() {
        System.out.println("**********Toggle Right Switch**********");
        if (m_rightSwitch == true) {
            m_rightSwitch = false;
        } else if (m_rightSwitch == false) {
            m_rightSwitch = true;
        }
    }

    public void setLeftSwitch(Boolean b) {
        m_leftSwitch_isRed = b;

        SmartDashboard.putBoolean("is Red Alliance", m_leftSwitch_isRed);
    }

    public void setMidSwitch(Boolean b) {
        m_isCube = b;

        SmartDashboard.putBoolean("is Cube", m_isCube);
    }

    public void setRightSwitch(Boolean b) {
        m_rightSwitch = b;
    }

    public void setButtonBox(Boolean b) {
        m_buttonBox = b;
        SmartDashboard.putBoolean("Button Box", m_buttonBox);
    }

    public void setDial(Long l) {
        m_dial = l;
    }
}
