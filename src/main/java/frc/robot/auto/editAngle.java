package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class editAngle extends Command {
    private long startingTime;
    private final DriveSubsystem m_drivetrainSubsystem;



    public editAngle(DriveSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;


        addRequirements(drivetrainSubsystem);
    }





    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();

    }


    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(0, 0, 0.1, false, false);
    }

    @Override
    public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(0, 0, 0.1, false, false);
    }




    private final int MaxTime = 1000;

    @Override
    public boolean isFinished() {
        long elapsedTime = System.currentTimeMillis() - startingTime;
        return elapsedTime > MaxTime ? true : false;
    }

}