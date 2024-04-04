package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class autoDrive extends Command {
    private long startingTime;
    private final DriveSubsystem m_drivetrainSubsystem;



    public autoDrive(DriveSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;


        addRequirements(drivetrainSubsystem);
    }





    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();

    }


    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(-0.5, 0, 0, false, false);
    }

    @Override
    public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(0, 0, 0, false, false);
    }




    private final int MaxTime = 3000;

    @Override
    public boolean isFinished() {
        long elapsedTime = System.currentTimeMillis() - startingTime;
        return elapsedTime > MaxTime ? true : false;
    }

}