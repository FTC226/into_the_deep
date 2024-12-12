package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

public class RunClaw {
    private Claw claw;

    public RunClaw(Claw claw){
        this.claw = claw;
    }

    public void init(){

    }

    public void loop(){
        if(gamepad2.dpad_up){
            claw.moveUp();
        } else if(gamepad2.dpad_down){
            claw.moveDown();
        }else if(gamepad2.dpad_left){
            claw.rotate(true);
        }else if(gamepad2.dpad_right){
            claw.rotate(false);
        }

        if(gamepad2.a){
            claw.bigOpen();
        } else{
            claw.bigClose();
        }
    }

}
