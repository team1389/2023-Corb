package frc.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;
import frc.subsystems.Arm.ArmPosition;

public class SetArmPosition extends CommandBase {
    private Arm arm;
    private ArmPosition target;
    private boolean runInstantly;
    double timeout = 2;
    Timer timer = new Timer();

    public SetArmPosition(Arm arm, ArmPosition targetPosition, boolean runInstantly) {
        this.arm = arm;
        this.runInstantly = runInstantly;
        target = targetPosition;

        addRequirements(arm);
    }   

    public SetArmPosition(Arm arm, ArmPosition targetPosition, boolean runInstantly, double timeout) {
       this(arm, targetPosition, runInstantly);

       this.timeout = timeout;
    }   

    @Override
    public void initialize() {
        arm.controllerInterrupt = false;
        arm.setArm(target);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("timer",timer.get());
    }

    @Override
    public boolean isFinished() {
        return runInstantly ^ timer.get() > timeout;
    }
}
