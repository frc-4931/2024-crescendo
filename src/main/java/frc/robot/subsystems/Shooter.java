package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;
    private boolean isRunning;
    private boolean isFast;
    private double speed = 0;//-0.95;
    private SparkPIDController pidController;
    private static final CANSparkLowLevel.MotorType kMotorType = CANSparkLowLevel.MotorType.kBrushless;

    public Shooter() {
        isRunning = false;
        isFast    = false;
        
        shootMotor1 = new CANSparkMax(10, kMotorType);
        shootMotor2 = new CANSparkMax(11, kMotorType);
        shootMotor1.restoreFactoryDefaults();
        shootMotor1.setIdleMode(IdleMode.kCoast);
        shootMotor2.setIdleMode(IdleMode.kCoast);
        shootMotor1.setSmartCurrentLimit(60);
        shootMotor2.setSmartCurrentLimit(60);
        // shootMotor1.setInverted(true);
        shootMotor1.follow(shootMotor2, true);
        

        pidController = shootMotor2.getPIDController();
        pidController.setP(6e-5);
        pidController.setFF(0.000182);
        pidController.setOutputRange(-1, 1);
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
    public Command stop() {
        return this.runOnce(() -> {
        isRunning = false;
        makeSpeed(0);});
    }
    /**
     * Ideal for amp
     * @param speed
     */
    public Command runSlow() {
        return this.runOnce(() -> {
        isRunning = true;
        isFast = false;
        makeSpeed(-0.09);});
    }  

    public Command runReverse() {
        return this.runOnce(() ->{
            isRunning = true;
            isFast = true;
            makeSpeed(-speed);
        });
    }

    /**
     * better for shooting upward and outward
     * @param speed
     */
    public void runFast() {
        isRunning = true;
        isFast = true;
        makeSpeed(-0.925);
    }
    /**
     * sets the speed of both motors, only accesable in this class
     * speed shall NEVER exceed domain (-1,1)
     * @param speed
     */
    private void makeSpeed(double speed) {
        this.speed = speed;

        // if(isRunning) {
        //     if(isFast) {
            // shootMotor1.set(speed);
            // shootMotor2.set(speed);
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
        super.periodic();
        // System.out.println(speed);
        pidController.setReference(speed*5700, ControlType.kVelocity);
        SmartDashboard.putNumber("setPointVel", speed*5700);
        SmartDashboard.putNumber("vel", shootMotor2.getEncoder().getVelocity());
        // var s = SmartDashboard.getNumber("shooter", 0);
        // if (s != speed) {
        //     speed = s;
        // }
        SmartDashboard.putBoolean("isOn", isRunning);
        SmartDashboard.putBoolean("isFast", isFast);
    }
}
