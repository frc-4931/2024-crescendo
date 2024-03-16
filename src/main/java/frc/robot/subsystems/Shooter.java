package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;
    private boolean isRunning;
    private boolean isFast;
    private double speed = -0.5;
    private static final CANSparkLowLevel.MotorType kMotorType = CANSparkLowLevel.MotorType.kBrushless;

    public Shooter() {
        isRunning = false;
        isFast    = false;
        shootMotor1 = new CANSparkMax(10, kMotorType);
        shootMotor2 = new CANSparkMax(11, kMotorType);
        SmartDashboard.putNumber("shooter", speed);
    }
    /**
     * to be used by RobotContainer. will toggle shooter
     * @param speed
     * @return
     */
    public Command toggleSlow() {
        return this.runOnce(() -> toggle(this::runSlow));
            // () -> {isFast = false; isRunning = !(isRunning); makeSpeed(speed/2);});
    }

    private void toggle(Runnable r) {
        if (isRunning) {
            stop();
         }
         else {
            r.run();
         }
    }

    /**
     * to be used by RobotContainer. will also toggle shooter, but faster
     * @param speed
     * @return
     */
    public Command toggleFast() {
        return this.runOnce(() -> toggle(this::runFast));
        //  {isFast = true; isRunning = !(isRunning); makeSpeed(speed);});
    }
    /**
     * stops the motors
     */
    public void stop() {
        isRunning = false;
        makeSpeed(0);
    }
    /**
     * Ideal for amp
     * @param speed
     */
    public void runSlow() {
        isRunning = true;
        isFast = false;
        makeSpeed(speed/2);
    }
    /**
     * better for shooting upward and outward
     * @param speed
     */
    public void runFast() {
        isRunning = true;
        isFast = true;
        makeSpeed(speed);
    }
    /**
     * sets the speed of both motors, only accesable in this class
     * speed shall NEVER exceed domain (-1,1)
     * @param speed
     */
    private void makeSpeed(double speed) {
        // if(isRunning) {
        //     if(isFast) {
            shootMotor1.set(speed);
            shootMotor2.set(speed);
        //     }
        //     else {
        //     shootMotor1.set(speed/2);
        //     shootMotor2.set(speed/2);
        //     }
        // }
        // else {
        //     shootMotor1.set(0);
        //     shootMotor2.set(0);
        // }
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        var s = SmartDashboard.getNumber("shooter", 0);
        if (s != speed) {
            speed = s;
        }
    }
}
