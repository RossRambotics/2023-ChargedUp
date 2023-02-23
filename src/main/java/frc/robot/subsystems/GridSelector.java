// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm.Arm;

/** Add your docs here. */
public class GridSelector extends SubsystemBase {
    private Joystick m_gridSelector = new Joystick(1);
    private Joystick m_gridSelector2 = new Joystick(2);
    private int m_dial = 0;

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

        SmartDashboard.putNumber("Grid Dial", m_dial);
    }
}
