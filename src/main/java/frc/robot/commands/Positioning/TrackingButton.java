// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positioning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.SnapDriveGamePiece;
import frc.robot.commands.Grabber.AutoGrab;
import frc.robot.commands.Tracking.EnableLight;

public class TrackingButton extends CommandBase {
    CommandBase m_cmd;

    /** Creates a new TrackingButton. */
    public TrackingButton() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        if (RobotContainer.m_GridSelector.isGridTracking()) {
            m_cmd = new GridStrafe();
        } else {

            m_cmd = new ParallelDeadlineGroup(new AutoGrab(),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> RobotContainer.m_grabber.openJaws()),
                            new SnapDriveGamePiece(RobotContainer.m_drivetrainSubsystem,
                                    () -> -RobotContainer.getTheRobot().getInputLeftY(),
                                    () -> -RobotContainer.getTheRobot().getInputLeftX(),
                                    () -> RobotContainer.m_Tracking.getTargetHeading()),
                            new EnableLight()));
        }
        m_cmd.schedule();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_cmd.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
