package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RumbleCommand {
    private static final double leftVal = 0.4;
    private static final double rightVal = 0.5;

    private XboxController controller;

    public RumbleCommand(XboxController controller) {
        this.controller = controller;
    }

    public Command leftCommand() {
        return command(() -> controller.setRumble(RumbleType.kLeftRumble, leftVal));
    }

    public Command rightCommand() {
        return command(() -> controller.setRumble(RumbleType.kRightRumble, rightVal));
    }

    public Command off() {
        return command(() -> controller.setRumble(RumbleType.kBothRumble, 0));
    }

    public Command pulse() {
        return new Command() {
            Timer timer = new Timer();
            int loop = 0;
            RumbleType rt = RumbleType.kLeftRumble;
            double v = leftVal;

            @Override
            public void initialize() {
                timer.reset();
                timer.start();
                loop = 0;
                controller.setRumble(rt, v);
                System.out.println("init");
            }

            @Override
            public void execute() {
                if (timer.hasElapsed(1 + loop)) {
                    controller.setRumble(rt, 0);
                }
                if (timer.hasElapsed(2 + loop)) {
                    controller.setRumble(rt, v);
                    loop += 2;
                }
            }

            

            @Override
            public boolean isFinished() {
                return timer.hasElapsed(10);
            }

            @Override
            public void end(boolean interrupted) {
                controller.setRumble(RumbleType.kBothRumble, 0);
            }
        };

    }

    private Command command(Runnable r) {
        return new InstantCommand(r);
    }
    
}
