package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bot;

public class Turret implements Subsystem {

    private DcMotorEx turret;
    private Bot bot;



    public Turret(Bot bot) {
        this.bot = bot;

        turret = bot.hMap.get(DcMotorEx.class,"Turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    @Override
    public void periodic(){

    }
    public void setPower(double power){
        power = power;
        turret.setPower(power);


    }
}