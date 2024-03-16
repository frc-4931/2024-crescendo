package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 public class Pneumatics extends SubsystemBase {

    private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    private DoubleSolenoid climber1, climber2, shelf1, shelf2, antenna;

    public Pneumatics() {
        climber1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
        climber2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
        shelf1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 6);
        shelf2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 8);
        antenna = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 9, 0);
    }
//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("air - analog voltage", compressor.getAnalogVoltage());
//         SmartDashboard.putNumber("air - pressure", compressor.getPressure());
//     }
    
}
