package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Bot;

public class Indexer implements Subsystem {

    private CRServo indexer;

    private Bot bot;
    public double power = 0;


    public Indexer(Bot bot) {
        this.bot = bot;

        indexer = bot.hMap.get(CRServo.class,"indexer");

    }


    @Override
    public void periodic(){

    }
    public void setPower(double power){
        power = power;
        indexer.setPower(power);


    }
}