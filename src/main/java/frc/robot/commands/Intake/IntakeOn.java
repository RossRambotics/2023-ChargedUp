// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeOn extends CommandBase {
    /** Creates a new IntakeOn. */
    public IntakeOn() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_intakeWheels);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_intakeWheels.intakeOn();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_intakeWheels.intakeOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
