package frc.robot.subsystems;

import javax.xml.validation.Validator;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shelf extends SubsystemBase {
    private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    private DoubleSolenoid climber1, climber2, shelf1, shelf2, antenna;


    public Shelf(){
        climber1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
        climber2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
        shelf1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 6);
        shelf2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 8);
        antenna = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 9, 0);
    }

    public Command openClimber() {
        return this.runOnce(() -> {
        climber1.set(Value.kReverse);
        climber2.set(Value.kReverse);
        });
    }

    public Command closeClimber() {
        return this.runOnce(() -> {
          climber1.set(Value.kForward);
          climber2.set(Value.kForward);
        });
      }

      public Command openShelf() {
        return this.runOnce(() -> {
          shelf1.set(Value.kReverse);
          shelf2.set(Value.kReverse);
        });
      }

      public Command closeShelf() {
        return this.runOnce(() -> {
          shelf1.set(Value.kForward);
          shelf2.set(Value.kForward);
        });
      }

       public Command openAntenna() {
        return this.runOnce(() -> {
          antenna.set(Value.kReverse);
        });
      }

      public Command closeAntenna() {
        return this.runOnce(() -> {
          antenna.set(Value.kForward);
        });
      }


    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
}
