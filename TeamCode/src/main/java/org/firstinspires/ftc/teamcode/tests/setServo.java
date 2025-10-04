package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.tests.setServo.targetServo.armPos;
import static org.firstinspires.ftc.teamcode.tests.setServo.targetServo.clawPivot;
import static org.firstinspires.ftc.teamcode.tests.setServo.targetServo.clawPos;
import static org.firstinspires.ftc.teamcode.tests.setServo.targetServo.clawPosUp;
import static org.firstinspires.ftc.teamcode.tests.setServo.targetServo.diffPosD;
import static org.firstinspires.ftc.teamcode.tests.setServo.targetServo.diffPosDUp;
import static org.firstinspires.ftc.teamcode.tests.setServo.targetServo.diffPosI;
import static org.firstinspires.ftc.teamcode.tests.setServo.targetServo.diffPosIUp;
import static org.firstinspires.ftc.teamcode.tests.setServo.targetServo.slidesPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Bot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SetServo", group = "Tools")
public class setServo extends CommandOpMode {
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    public void initialize() {

        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);


        telem.addData("status","init");
        telem.update();

    }

    @Config
    public static class targetServo{
        public static double clawPos = 0;
        public static double clawPivot = 0;
        public static double slidesPos = 0;
        public static double diffPosI = 0;
        public static double diffPosD = 0;
        public static double diffPosIUp = 0;
        public static double diffPosDUp = 0;
        public static double clawPosUp = 0;
        public static double armPos = 0;

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();




        telem.addData("status","start");
    }
}