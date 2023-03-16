// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.SnapDrive;

public class RedDriveUpChargeStation extends CommandBase {

    private CommandBase m_cmd;
    private Boolean m_isAway = false;
    private Boolean m_isBursted = false;

    private CommandBase cmdAwayBurst = new SnapDrive(RobotContainer.m_drivetrainSubsystem,
            () -> {
                return -0.3;
            },
            () -> {
                return -0.0;
            },
            0)
            .withTimeout(0.2).andThen(new WaitCommand(0.75));

    private CommandBase cmdTowardsBurst = new SnapDrive(RobotContainer.m_drivetrainSubsystem,
            () -> {
                return 0.3;
            },
            () -> {
                return -0.0;
            },
            0)
            .withTimeout(0.2).andThen(new WaitCommand(0.75));

    /** Creates a new DriveUpChargeStation. */
    public RedDriveUpChargeStation() {
        // Use addRequirements() here to declare subsystem dependencies.
        // this.addRequirements(RobotContainer.m_drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        AutoPoses.SetStartPose(AutoPoses.RedFive);
        m_cmd = new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> {
                    if (RobotContainer.m_drivetrainSubsystem.getPitch() > 5.0)
                        return true;
                    return false;
                }),
                new SnapDrive(RobotContainer.m_drivetrainSubsystem,
                        () -> {
                            return -0.75;
                        },
                        () -> {
                            return -0.0;
                        },
                        0),
                new PrintCommand("******** Initialize Phase 1"))
                .andThen(new WaitCommand(0.2)).andThen(new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> {
                            if (RobotContainer.m_drivetrainSubsystem.getPitch() > 10.0)
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
                                0),
                        new PrintCommand("******** Initialize Phase 2")));

        m_cmd.setName("Inititalize");
        m_cmd.schedule();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // SmartDashboard.putString("BalanceMsg", "Running command: " +
        // m_cmd.getName());
        System.out.println("*********Running command: " + m_cmd.getName());

        if (m_cmd.isScheduled()) {
            return;
        }

        if (!m_isBursted) {
            // do burst the other way
            m_isBursted = true;
            if (m_isAway) {
                m_cmd = cmdTowardsBurst;
                m_cmd.schedule();
            } else {
                m_cmd = cmdAwayBurst;
                m_cmd.schedule();
            }
        }

        // Check if Balanced
        double pitch = RobotContainer.m_drivetrainSubsystem.getPitch();
        if (Math.abs(pitch) < 5.0) {
            return;
        }

        // need to go away from grid...?
        if (pitch < -2.0) {
            m_isAway = true;
            m_isBursted = false;
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
                    .andThen(new WaitCommand(1))
                    .withTimeout(0.2);
            m_cmd.setName("Balance Away from Grid");
            m_cmd.schedule();
        } else {
            m_isAway = false;
            m_isBursted = false;
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
                    .andThen(new WaitCommand(1))
                    .withTimeout(0.2);
            m_cmd.setName("Balance Towards Grid");
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
