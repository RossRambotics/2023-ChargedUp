// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWheels extends SubsystemBase {

    WPI_TalonFX m_leftMotor = new WPI_TalonFX(Constants.INTAKE_LEFT_MOTOR, "usb");
    WPI_TalonFX m_rightMotor = new WPI_TalonFX(Constants.INTAKE_RIGHT_MOTOR, "usb");
    private int m_stallCounter = 0;

    /** Creates a new IntakeWheels. */
    public IntakeWheels() {
        m_leftMotor.configFactoryDefault();
        m_rightMotor.configFactoryDefault();

        m_rightMotor.setInverted(true);
        m_leftMotor.setInverted(false);
        m_rightMotor.follow(m_leftMotor);
    }

    public void intakeOn() {
        m_leftMotor.setVoltage(3.0);
        m_stallCounter = 0;
    }

    public void intakeOff() {
        m_leftMotor.setVoltage(0.0);
    }

    public boolean isStalled() {
        double vel = Math.abs(m_leftMotor.getSelectedSensorVelocity());

        if (vel < 30) {
            m_stallCounter++;
        }

        if (m_stallCounter > 10) {
            return true;
        }

        return false;
    }

    public void intakeReverse() {
        m_leftMotor.setVoltage(-3.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
