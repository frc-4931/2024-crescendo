package frc.robot.subsystems;

import javax.xml.validation.Validator;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shelf extends SubsystemBase {
    private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    private DoubleSolenoid climber1, climber2, shelf, antenna;
    private boolean antennaOn, shelfOn, climberOn;


    public Shelf(){
      
        climber1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 6);
        climber2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 2);
        shelf    = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        antenna  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
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
          shelf.set(Value.kReverse);
        });
      }

      public Command closeShelf() {
        return this.runOnce(() -> {
          shelf.set(Value.kForward);
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

      public Command toggleAntenna() {
        
        return this.runOnce(() -> { antennaOn = !antennaOn;
        antenna.set(antennaOn ? Value.kForward : Value.kReverse);
        });
      }
      public Command toggleShelf() {
      
        return this.runOnce(() -> { shelfOn = !shelfOn;
        shelf.set(shelfOn ? Value.kForward : Value.kReverse);
        });
      }
      public Command toggleClimbers() {
        
        return this.runOnce(() -> { climberOn = !climberOn;
        climber1.set(climberOn ? Value.kForward : Value.kReverse);
        climber2.set(climberOn ? Value.kForward : Value.kReverse);
        });
      }

      public Command shelfAntennaDrop () {
        return closeAntenna().andThen
        (Commands.waitSeconds(0.5)).andThen(
          openShelf()
        );
      }


    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
}
