package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ShooterSubsystem;


public class multiNote extends Command {
    private long startingTime;
    private final DriveSubsystem m_drivetrainSubsystem;
    // Shooter 
    // Intake Wheels
    // Arm
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final IntakeWheels m_intakeWheels;


    public multiNote(DriveSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, IntakeWheels intakeWheels) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_shooterSubsystem = shooterSubsystem;
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_intakeWheels = intakeWheels;

        addRequirements(drivetrainSubsystem, shooterSubsystem, intakeSubsystem, intakeWheels);
    }





    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();

    }


    @Override
    public void execute() {
        long elapsedTime = System.currentTimeMillis() - startingTime;
        // First, eject the note into the speaker, then turn around, then put the arm down, then drive forward and run the intake, then bring the arm back, then do a 180, then drive forward for one second, then eject the note
        if (elapsedTime < 1000) {
            m_shooterSubsystem.set_speed(0.90);
            m_intakeWheels.set_speed(-0.5);
        }
        else if (elapsedTime < 2000) {
            m_shooterSubsystem.set_speed(0);
            m_intakeWheels.set_speed(0);
            m_drivetrainSubsystem.drive(0, 0, 0.5, false, false);
        }
        else if (elapsedTime < 3000) {
            //if (m_intakeSubsystem.getAngle() < Constants.ArmConstants.max_limit) {
              //  m_intakeSubsystem.set_speed(0.4);
            //}
        }
        else if (elapsedTime < 4000) {
        m_drivetrainSubsystem.drive(0.5, 0, 0, false, false);
        if (m_intakeSubsystem.hasRing() == false) {
            m_intakeWheels.set_speed(0.5);
        }
        }
        else if (elapsedTime < 5000) {
        m_intakeWheels.set_speed(0);
        //if (m_intakeSubsystem.getAngle() > Constants.ArmConstants.min_limit) {
          //      m_intakeSubsystem.set_speed(-0.4);
            //}
        }
        else if (elapsedTime < 6000) {
            m_drivetrainSubsystem.drive(0, 0, 0.5, false, false);
        }
        else if (elapsedTime < 7000) {
            m_drivetrainSubsystem.drive(0.2, 0, 0, false, false);
        }
        else if (elapsedTime < 8000) {
             m_shooterSubsystem.set_speed(0.90);
            m_intakeWheels.set_speed(-0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(0, 0, 0.0, false, false);
    m_shooterSubsystem.set_speed(0);
    m_intakeWheels.set_speed(0);
    m_intakeSubsystem.set_speed(0);

    }




    private final int MaxTime = 8500;

    @Override
    public boolean isFinished() {
        long elapsedTime = System.currentTimeMillis() - startingTime;
        return elapsedTime > MaxTime ? true : false;
    }

}