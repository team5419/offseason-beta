package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DoUntilCommand extends Command {

    Command onTrue;
    Command onFalse;
    BooleanSupplier condition;

    public DoUntilCommand(Command onTrue, Command onFalse, BooleanSupplier condition){
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        this.condition = condition;
        addRequirements(onTrue.getRequirements());
    }
    public void init(){
        onTrue.initialize();
        onFalse.initialize();
    }
    public void execute(){
        if(condition.getAsBoolean()){
            onTrue.execute();
        } else{
            onFalse.execute();
        }
    }
    public boolean isFinished(){
            return onFalse.isFinished() || (onTrue.isFinished() && !condition.getAsBoolean());
    }
    public void end(){
        
    }
    
}
