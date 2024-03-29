// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AnalogInput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Grabber extends SubsystemBase {
    public edu.wpi.first.wpilibj.AnalogInput m_Sensor_Grabber = new edu.wpi.first.wpilibj.AnalogInput(
            Constants.GRABBER_SENSOR);
    Compressor m_compressor = new Compressor(frc.robot.Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH);
    DoubleSolenoid m_grabDoubleSolenoid = new DoubleSolenoid(frc.robot.Constants.PNEUMATIC_HUB,
            PneumaticsModuleType.REVPH, 0, 1);

    /** Creates a new Grabber. */
    public Grabber() {

    }

    public void startCompresser() {
        m_compressor.enableAnalog(80, 110);
    }

    public void openJaws() {
        m_grabDoubleSolenoid.set(Value.kForward);

        Pose2d current = RobotContainer.m_drivetrainSubsystem.getOdometryPose();
        DataLogManager.log("Grabber Open: Grid Dial: " + (RobotContainer.m_GridSelector.getDial() + 1)
                + " Current Pose: " + current);
    }

    public void closeJaws() {
        m_grabDoubleSolenoid.set(Value.kReverse);
        Pose2d current = RobotContainer.m_drivetrainSubsystem.getOdometryPose();

        DataLogManager.log("Grabber Close: Current Pose: " + current);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Grabber Sensor", this.m_Sensor_Grabber.getValue());
        // This method will be called once per scheduler run
    }

    public boolean getSensorGrabber() {
        if (m_Sensor_Grabber.getValue() < 10) {
            return true;
        } else {
            return false;
        }
    }

    public void createShuffleBoardTab() {

    }
}