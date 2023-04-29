// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.WaitOnArm;
import frc.robot.commands.Drive.SnapDriveGamePiece;
import frc.robot.commands.Drive.SnapDriveToPoseField;
import frc.robot.commands.Grabber.AutoGrab;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.IntakeOn;
import frc.robot.commands.Tracking.EnableLight;
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
                                Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.C),
                                new WaitOnArm(),
                                Commands.runOnce(() -> RobotContainer.m_grabber.openJaws()),
                                Arm.targetNodeCommandFactory(RobotContainer.m_arm,
                                                RobotContainer.m_arm.YY))
                                .andThen(SnapDriveToPoseField.createRelative(AutoPoses.RedNine, -3.9, 0, 0, 0.10))
                                .andThen(new ExtendIntake())
                                .andThen(new IntakeOn())
                                .andThen(AutoPoses.DriveToPose(
                                                AutoPoses.GP_RedNine))
                                .andThen(new ParallelDeadlineGroup(new AutoGrab(),
                                                new ParallelCommandGroup(
                                                                new SnapDriveGamePiece(
                                                                                RobotContainer.m_drivetrainSubsystem,
                                                                                () -> 0.0,
                                                                                () -> 0.0,
                                                                                () -> RobotContainer.m_Tracking
                                                                                                .getTargetHeading()),
                                                                new EnableLight())))
                                .andThen(new WaitCommand(0.75))
                                .andThen(Arm.targetNodeCommandFactory(RobotContainer.m_arm,
                                                RobotContainer.m_arm.N))
                                .andThen(new WaitCommand(0.5))
                                .andThen(new SnapDriveToPoseField(RobotContainer.m_drivetrainSubsystem,
                                                AutoPoses.RedNine,
                                                0.1))
                                .andThen(new SnapDriveToPoseField(RobotContainer.m_drivetrainSubsystem,
                                                AutoPoses.RedSix,
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
