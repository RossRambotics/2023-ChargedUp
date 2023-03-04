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
    private boolean m_leftSwitch = true;
    private boolean m_midSwitch = true;
    private boolean m_rightSwitch = true;
    private boolean m_buttonBox = true;

    /** Creates a new GridSelector2. */
    public GridSelector() {

    }

    public void initialize() {
        Trigger btnHoldHigh = new JoystickButton(m_gridSelector2, 2);
        CommandBase cmd = new PrintCommand("************** Button Pressed! *******************");
        cmd = Arm.setpointCommandFactory("hold high", 0, 45, 1);
        btnHoldHigh.onTrue(cmd);

        Trigger btnMidpoint = new JoystickButton(m_gridSelector2, 5);
        cmd = Arm.setpointCommandFactory("Midpoint", -45, 45, 1);
        btnMidpoint.onTrue(cmd);

        Trigger btnPickUp = new JoystickButton(m_gridSelector2, 8);
        cmd = Arm.setpointCommandFactory("Pick up", -90, 80, 1);
        btnPickUp.onTrue(cmd);

        Trigger btnCarry = new JoystickButton(m_gridSelector2, 6);
        cmd = Arm.setpointCommandFactory("Carry", -120, 110, 1);
        btnCarry.onTrue(cmd);

        Trigger btnEnd = new JoystickButton(m_gridSelector2, 3);
        cmd = Arm.setpointCommandFactory("End", 0, 0, 1);
        btnEnd.onTrue(cmd);

        Trigger btnSafeForward = new JoystickButton(m_gridSelector2, 4);
        cmd = Arm.setpointCommandFactory("safe forward", -45, 120, 1);
        btnSafeForward.onTrue(cmd);

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
                m_leftSwitch = true;
            } else if (m_gridSelector.getRawAxis(0) < -.5) {
                // Red and Blue switch on Blue
                m_leftSwitch = false;
            }

            if (m_gridSelector.getRawAxis(1) < -.5) {
                // Cube and Cone switch on Cube
                m_midSwitch = true;
            } else if (m_gridSelector.getRawAxis(0) > -.5) {
                // Cube and Cone switch on Cone
                m_midSwitch = false;
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
            m_leftSwitch = SmartDashboard.getBoolean("Left Switch", false);
            m_midSwitch = SmartDashboard.getBoolean("Mid Switch", false);
        }

        SmartDashboard.putBoolean("Left Switch", m_leftSwitch);
        SmartDashboard.putBoolean("Mid Switch", m_midSwitch);
        SmartDashboard.putBoolean("Right Switch", m_rightSwitch);
        SmartDashboard.putBoolean("Button Box", m_buttonBox);

    }

    public Boolean getLeftSwitch() {
        return m_leftSwitch;
    }

    public Boolean getMidSwitch() {
        return m_midSwitch;
    }

    public Boolean getRightSwitch() {
        return m_rightSwitch;
    }

    public void toggleLeftSwitch() {
        System.out.println("**********Toggle Left Switch**********");
        if (m_leftSwitch == true) {
            m_leftSwitch = false;
        } else if (m_leftSwitch == false) {
            m_leftSwitch = true;
        }
    }

    public void toggleMidSwitch() {
        System.out.println("**********Toggle Mid Switch**********");
        if (m_midSwitch == true) {
            m_midSwitch = false;
        } else if (m_midSwitch == false) {
            m_midSwitch = true;
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
        m_leftSwitch = b;
    }

    public void setMidSwitch(Boolean b) {
        m_midSwitch = b;
    }

    public void setRightSwitch(Boolean b) {
        m_rightSwitch = b;
    }

    public void setButtonBox(Boolean b) {
        m_buttonBox = b;
    }

    public void setDial(Long l) {
        m_dial = l;
    }
}
