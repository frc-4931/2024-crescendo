package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase{
     private CANSparkMax conveyMotor1;
    private CANSparkMax conveyMotor2;
    private boolean isRunning;
    private static final CANSparkLowLevel.MotorType kMotorType = CANSparkLowLevel.MotorType.kBrushless;

    public Conveyor() {
        isRunning = false;
        conveyMotor1 = new CANSparkMax(12, kMotorType);
        conveyMotor2 = new CANSparkMax(13, kMotorType);
    }
    /**
     * to be used by RobotContainer. will toggle shooter
     * @param speed
     * @return
     */
    public Command toggle(double speed) {
        return this.runOnce(() -> { isRunning = !(isRunning); makeSpeed(speed);});
    }
    /**
     * stops the motors
     */
    public void stop() {
        isRunning = false;
        makeSpeed(0);
    }
    /**
     * moves up to shooter
     * @param speed
     */
    public void run(double speed) {
        isRunning = true;
        makeSpeed(speed);
    }
    /**
     * sets the speed of both motors, only accesable in this class
     * speed shall NEVER exceed domain (-1,1)
     * @param speed
     */
    private void makeSpeed(double speed) {
        if(isRunning) {
            conveyMotor1.set(speed);
            conveyMotor2.set(speed);
        }
        else {
            conveyMotor1.set(0);
            conveyMotor2.set(0);
        }
    }

}
