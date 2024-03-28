
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class IntakeSubsystem extends SubsystemBase {

private CANSparkMax  motorController1;
private CANSparkMax  motorController2;
private DutyCycleEncoder m_DutyCycleEncoder;
private DigitalInput dioInput;

    public IntakeSubsystem() {
        dioInput = new DigitalInput(1);
        m_DutyCycleEncoder = new DutyCycleEncoder(0);
        motorController1 = new CANSparkMax(9, MotorType.kBrushless);
        motorController1.setInverted(true);
        motorController2 = new CANSparkMax(10, MotorType.kBrushless);
        motorController2.setInverted(false);
        motorController1.setIdleMode(IdleMode.kBrake);
        motorController2.setIdleMode(IdleMode.kBrake);
        m_DutyCycleEncoder.reset();
    }   

     // New number system, called rodegrees (robot degrees) such that the max limit is 180
    // Actaully dont mind the above comment dave insists on using the pivot of the arm as the orign.
        public double getAngle() { 
        return m_DutyCycleEncoder.get() * 120;
    }


  // High precision  acceleration control H.P.A.C
    public void set_speed(double value){
       System.out.println("Attempting to set speed: Value" + "[" + value + "]");
        System.out.println("Current Angle: " + getAngle());

        if (value > 0 && getAngle() < Constants.ArmConstants.max_limit) {
            System.out.println("Setting speed Positive [full] (up)");

            // Arm trying to go up
            if (Constants.ArmConstants.max_limit - getAngle() < 30) {
                //Arm is close to max limit
                System.out.println("Setting speed Positive [reduced] (up)");
                motorController1.set(0.1);
                motorController2.set(0.1);
            }
            else {
                //Arm is not close to max limit
                motorController1.set(value);
                motorController2.set(value);
            }
        }
        else if (value < 0 && getAngle() > Constants.ArmConstants.min_limit) {
        System.out.println("Setting speed Negative [full] (down)");
        // If the arm is near the min limit, reduce the speed
        if (getAngle() <= Constants.ArmConstants.min_limit + 30) {
                //Arm is close to min limit
                System.out.println("Setting speed Negative [reduced] (down)");
                motorController1.set(-0.1);
                motorController2.set(-0.1);
            }
            else {
                //Arm is not close to min limit
                motorController1.set(value);
                motorController2.set(value);
            }
        }
        else {
            System.out.println("Setting speed 0 (stop)");
            motorController1.set(0);
            motorController2.set(0);
        
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public boolean hasRing() {
        boolean isHigh = dioInput.get();

        if (!isHigh) {
            // The DIO port is receiving high voltage
            return true;
        } else {
            // The DIO port is receiving low voltage
            return false;
            }
    }
    


   
   
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}