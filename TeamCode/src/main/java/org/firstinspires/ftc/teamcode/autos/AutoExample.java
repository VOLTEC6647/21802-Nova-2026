
package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




@Config
@Autonomous
public class AutoExample extends LinearOpMode {

    // Scoring Poses
    public static Pose startingPose = new Pose(9, 87, Math.toRadians(0));
    public static Pose chamberPose = new Pose(36, 75, Math.toRadians(0));


    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();


        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        VoltageSensor vs = bot.hMap.voltageSensor.iterator().next();

        Follower f = Constants.createFollower(bot.hMap);
        f.setStartingPose(startingPose);
        f.update();



        SequentialCommandGroup auto = new SequentialCommandGroup(

                new SequentialCommandGroup(

                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        startingPose,
                                                        chamberPose
                                                )
                                        )
                                        .setConstantHeadingInterpolation(chamberPose.getHeading())
                                        .build()
                                )
                        )

                )
        );

        waitForStart();
        CommandScheduler.getInstance().schedule(auto);

        while (opModeIsActive()) {
            f.setMaxPower(10.0 / vs.getVoltage());
            CommandScheduler.getInstance().run();
            f.update();

        }
    }
}
