// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {

    private final ShooterSubsystem m_shooter;
    private final IntakeWheels m_intakeWheels;

    public Shoot(ShooterSubsystem subsystem, IntakeWheels intakeWheels) {
        m_shooter = subsystem;
        m_intakeWheels = intakeWheels;
        addRequirements(m_shooter, m_intakeWheels);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooter.set_speed(0.95);
        m_intakeWheels.set_speed(-0.5);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // TODO Find the correct speed for the shooter to shoot for the speaker
        // System.out.println("Shooter speed set to 0.5");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.set_speed(0);
        m_intakeWheels.set_speed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}