// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.WaitOnArm;
import frc.robot.commands.Drive.SnapDriveGamePiece;
import frc.robot.commands.Drive.SnapDriveToPoseField;
import frc.robot.commands.Grabber.AutoGrab;
import frc.robot.commands.Tracking.EnableLight;
import frc.robot.subsystems.Arm.Arm;

public class AutoBlueOne extends CommandBase {
    /** Creates a new AutoBlueOne. */
    public AutoBlueOne() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Log the auto command name
        DataLogManager.log("Auto command: " + this.getName());

        // Set Starting Pose
        AutoPoses.SetStartPose(AutoPoses.BlueOne);

        // Create command group for the auto routine
        SequentialCommandGroup command = new SequentialCommandGroup(
                new WaitCommand(2.0),
                Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.W),
                new WaitOnArm(),
                new WaitCommand(0.5),
                Commands.runOnce(() -> RobotContainer.m_grabber.closeJaws()),
                new WaitCommand(0.75),
                Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.C),
                new WaitOnArm(),
                Commands.runOnce(() -> RobotContainer.m_grabber.openJaws()),
                Arm.targetNodeCommandFactory(RobotContainer.m_arm,
                        RobotContainer.m_arm.O))
                .andThen(SnapDriveToPoseField.createRelative(
                        AutoPoses.BlueOne, 3.9, 0, 0, 0.10))
                .andThen(AutoPoses.DriveToPose(
                        AutoPoses.GP_BlueOne))
                .andThen(new ParallelDeadlineGroup(new AutoGrab(),
                        new ParallelCommandGroup(
                                new SnapDriveGamePiece(
                                        RobotContainer.m_drivetrainSubsystem,
                                        () -> 0.0,
                                        () -> 0.0,
                                        () -> RobotContainer.m_Tracking.getTargetHeading()),
                                new EnableLight())))
                .andThen(new WaitCommand(0.75))
                .andThen(Arm.targetNodeCommandFactory(RobotContainer.m_arm,
                        RobotContainer.m_arm.N))
                .andThen(new WaitCommand(0.5))
                .andThen(new SnapDriveToPoseField(RobotContainer.m_drivetrainSubsystem,
                        AutoPoses.BlueOne,
                        0.1))
                .andThen(new SnapDriveToPoseField(RobotContainer.m_drivetrainSubsystem,
                        AutoPoses.BlueThree,
                        0.1));

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
