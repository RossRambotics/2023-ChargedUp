// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoGrab extends CommandBase {
  /** Creates a new AutoGrab. */
  private boolean m_isdone = false;

  public AutoGrab() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(RobotContainer.m_grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_grabber.openJaws();
    m_isdone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_grabber.getSensorGrabber() == true) {
      RobotContainer.m_grabber.closeJaws();
      m_isdone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isdone;
  }
}
