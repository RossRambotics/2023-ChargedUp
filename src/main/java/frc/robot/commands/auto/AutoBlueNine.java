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
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.IntakeOn;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Tracking.EnableLight;
import frc.robot.subsystems.Arm.Arm;

public class AutoBlueNine extends CommandBase {
    /** Creates a new AutoBlueNine. */
    public AutoBlueNine() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Log the auto command name
        DataLogManager.log("Auto command: " + this.getName());

        // Set Starting Pose
        AutoPoses.SetStartPose(AutoPoses.BlueNine);

        // Create command group for the auto routine

        SequentialCommandGroup command =
                // Place the cone
                Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.C)
                        .andThen(new WaitOnArm())

                        // drop off the cone high
                        .andThen(Commands.runOnce(() -> RobotContainer.m_grabber.openJaws()))

                        // retract the arm to carry
                        .andThen(Arm.targetNodeCommandFactory(RobotContainer.m_arm, RobotContainer.m_arm.A))

                        // extend the intake, turn it on and grab a cube
                        // raceWith ends when it reaches the pose OR picked up the cube
                        .andThen(new ExtendIntake())
                        .andThen(new IntakeOn()
                                .raceWith(AutoPoses.DriveToPose(AutoPoses.GP_BlueNine)))

                        // retract the intake so it is protected by the bumpers when we turn around
                        .andThen(new RetractIntake())

                        // drive back to original pose, stop a little short and turn around
                        .andThen(SnapDriveToPoseField.createRelative(AutoPoses.BlueNine, 0.3, 0.4, 180, 0.1))

                        // spit out the cube
                        .andThen(new ExtendIntake())
                        .andThen(new IntakeReverse().withTimeout(1.5));

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
