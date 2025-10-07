package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Bot;

public class Shooter implements Subsystem {

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    private Bot bot;
    public double power = 0;


    public Shooter(Bot bot) {
        this.bot = bot;

        motor1 = bot.hMap.get(DcMotorEx.class, "motor1");
        motor2 = bot.hMap.get(DcMotorEx.class, "motor2");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);


    }


    @Override
    public void periodic(){

    }
    public void setPower(double power){
        power = power;
        motor1.setPower(power);


    }
}