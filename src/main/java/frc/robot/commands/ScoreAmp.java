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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeWheels;

// We are assuming that the arm will already be in the correct position to intake the ring

// Only use this command when about to pick up a ring
public class ScoreAmp extends Command {
    // This will be the amount of time we'll set the wheels to run AFTER we detect a ring
    private final IntakeWheels m_intakeWheels;
    private final IntakeSubsystem m_intakeSubsystem;
    private long startingTime;

    public ScoreAmp(IntakeWheels subsystem, IntakeSubsystem intakeSubsystem) {
        m_intakeWheels = subsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeWheels, intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();
    }
    private int MaxTime = 3000; //Force robot to exit this command after this much time (milliseconds)
    // Called every time the scheduler runs while the command is scheduled.

    // This is our accurate automatic intake system A.A.I.S
    @Override
    public void execute() {


        long elapsedTime = System.currentTimeMillis() - startingTime;

        if (elapsedTime < MaxTime) {
        if (m_intakeSubsystem.getAngle() < 88) { // 2 degrees less than 90
            m_intakeSubsystem.set_speed(0.5);
        } else if (m_intakeSubsystem.getAngle() > 92) { // 2 degrees more than 90
            m_intakeSubsystem.set_speed(-0.5);
        } else {
            m_intakeWheels.set_speed(-0.5);
        }
        } else {
            m_intakeWheels.set_speed(0);
        }


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Here we will stop the intake wheels
        m_intakeWheels.set_speed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        long elapsedTime2 = System.currentTimeMillis() - startingTime;
        if (elapsedTime2 > MaxTime) {
            return false;
        }
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}