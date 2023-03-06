// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class LowerArmSetPoint extends CommandBase {
  private double m_radians = 0.0;
  private double m_tolerance = 0.0;

  /** Creates a new UpperArm. */
  public LowerArmSetPoint(double radians, double toleranceRadians) {
    // Use addRequirements() here to declare subsystem dependencies.
    // this.addRequirements(RobotContainer.m_lowerArm);

    m_radians = radians;
    m_tolerance = toleranceRadians;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_lowerArm.getController().setTolerance(m_tolerance);
    RobotContainer.m_lowerArm.setGoal(m_radians);
    RobotContainer.m_lowerArm.enable();
    DataLogManager.log("LowerArmSetPoint: " + m_radians + " Tolerance: " + m_tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_upperArm.getController().atGoal();
  }
}
