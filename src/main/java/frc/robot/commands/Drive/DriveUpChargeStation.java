// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class DriveUpChargeStation extends CommandBase {

  private CommandBase m_cmd;

  /** Creates a new DriveUpChargeStation. */
  public DriveUpChargeStation() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(RobotContainer.m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cmd = new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> {
          if (RobotContainer.m_drivetrainSubsystem.getPitch() > 5.0)
            return true;
          return false;
        }),
        new SnapDrive(RobotContainer.m_drivetrainSubsystem,
            () -> {
              return -0.5;
            },
            () -> {
              return -0.0;
            },
            0))
        .andThen(new WaitCommand(0.2)).andThen(new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> {
              if (RobotContainer.m_drivetrainSubsystem.getPitch() < 10.0)
                return true;
              return false;
            }),
            new SnapDrive(RobotContainer.m_drivetrainSubsystem,
                () -> {
                  return 0.30;
                },
                () -> {
                  return -0.0;
                },
                0)));

    m_cmd.setName("Drive Up Charge Station");
    m_cmd.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_cmd.isFinished()) {
      return;
    }

    // Check if Balanced
    double pitch = RobotContainer.m_drivetrainSubsystem.getPitch();
    if (Math.abs(pitch) < 2.0) {
      return;
    }

    // need to go away from charge station...?
    if (pitch < -2.0) {
      m_cmd = new ParallelDeadlineGroup(
          new WaitUntilCommand(() -> {
            if (RobotContainer.m_drivetrainSubsystem.getPitch() > 0.0)
              return true;
            return false;
          }),
          new SnapDrive(RobotContainer.m_drivetrainSubsystem,
              () -> {
                return -0.3;
              },
              () -> {
                return -0.0;
              },
              0))
          .andThen(new WaitCommand(0.1))
          .withTimeout(0.2);
      m_cmd.schedule();
    } else {
      m_cmd = new ParallelDeadlineGroup(
          new WaitUntilCommand(() -> {
            if (RobotContainer.m_drivetrainSubsystem.getPitch() < 0.0)
              return true;
            return false;
          }),
          new SnapDrive(RobotContainer.m_drivetrainSubsystem,
              () -> {
                return 0.3;
              },
              () -> {
                return -0.0;
              },
              0))
          .andThen(new WaitCommand(0.1))
          .withTimeout(0.2);
      m_cmd.schedule();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
