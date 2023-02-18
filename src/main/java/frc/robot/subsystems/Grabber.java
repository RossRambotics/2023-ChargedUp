// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
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
  }

  public void closeJaws() {
    m_grabDoubleSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}