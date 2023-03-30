// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeFrame extends SubsystemBase {

    public edu.wpi.first.wpilibj.AnalogInput m_intakeSensor = new edu.wpi.first.wpilibj.AnalogInput(
            Constants.INTAKE_SENSOR);
    DoubleSolenoid m_grabDoubleSolenoid = new DoubleSolenoid(frc.robot.Constants.PNEUMATIC_HUB,
            PneumaticsModuleType.REVPH, 2, 3);

    /** Creates a new Intake. */
    public IntakeFrame() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void retract() {
        m_grabDoubleSolenoid.set(Value.kForward);
    }

    public void extend() {
        m_grabDoubleSolenoid.set(Value.kReverse);

    }

    public boolean hasGamePiece() {

        if (m_intakeSensor.getValue() < 10) {

            return true;
        } else {

            return false;
        }
    }
}
