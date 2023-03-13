// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.Arm;

public class AutoRedNine extends CommandBase {
  /** Creates a new AutoRedNine. */
  public AutoRedNine() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("Auto command: " + this.getName());

    AutoPoses.SetStartPose(AutoPoses.RedNine);

    SequentialCommandGroup command = new SequentialCommandGroup(
        Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.W),
        new WaitCommand(2),
        Commands.runOnce(() -> RobotContainer.m_grabber.closeJaws()),
        new WaitCommand(.5),
        Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.C),
        new WaitCommand(5),
        Commands.runOnce(() -> RobotContainer.m_grabber.openJaws()),
        AutoPoses.DriveToPose(AutoPoses.RedNineBack),
        Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.A),
        new WaitCommand(5));

    command.schedule();
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
    return true;
  }
}
