// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WaitOnArm extends CommandBase {
  boolean m_isFinished = false;

  /** Creates a new WaitOnArm. */
  public WaitOnArm() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_arm.isAtTarget()) {
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
