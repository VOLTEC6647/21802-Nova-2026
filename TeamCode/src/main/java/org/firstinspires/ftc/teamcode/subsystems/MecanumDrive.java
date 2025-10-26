package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Vision;
import org.firstinspires.ftc.teamcode.autos.AutoExample;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public class MecanumDrive extends SubsystemBase {
    private final Bot bot;
    private final IMU imu = null;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public static GoBildaPinpointDriver odo;
    public static Pose pose;
    public static Pose2D pose2D;
    private Vision vision;
    public static Pose startingPose = new Pose(56,8,0, PedroCoordinates.INSTANCE);
    public static Pose parkPose = new Pose(38.4, 33.8, Math.toRadians(0));
    private final Follower f;
    private final SequentialCommandGroup autoPark;


    public Rotation2d getRobotOrientation() {
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public MecanumDrive(Bot bot) {
        this.bot = bot;

        vision = new Vision(bot);
        vision.register();

        f = Constants.createFollower(bot.hMap);
        f.setStartingPose(startingPose);
        f.update();

        autoPark = new SequentialCommandGroup(

                new SequentialCommandGroup(

                        new ParallelCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        f.getPose(),
                                                        parkPose
                                                )
                                        )
                                        .setConstantHeadingInterpolation(parkPose.getHeading())
                                        .build()
                                )
                        )

                )
        );


        odo = bot.hMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(-82.66924000028, 110.830759999962, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if (pose == null) {
            pose2D = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0);
        }
        odo.setPosition(pose2D);

        frontLeft = bot.hMap.get(DcMotorEx.class, "FL");
        frontRight = bot.hMap.get(DcMotorEx.class, "FR");
        backLeft = bot.hMap.get(DcMotorEx.class, "BL");
        backRight = bot.hMap.get(DcMotorEx.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD );

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }




    @Override
    public void periodic() {

        if(bot.driver.gamepad.left_bumper){
            teleopDrive(0,0, vision.getTurnPower(), 1);
        }
        else {
            teleopDrive(0,0,0,1);
        }

        if (bot.driver.gamepad.start){
            odo.recalibrateIMU();
        }
        if (bot.driver.gamepad.dpad_left){
            CommandScheduler.getInstance().schedule(autoPark);
        }
        if (bot.driver.gamepad.dpad_right) {
            f.startTeleopDrive();
        }





        Pose position = new Pose(odo.getEncoderX(), odo.getEncoderY(), odo.getHeading(AngleUnit.RADIANS));

        odo.update();
        pose = position;


    }

    public void teleopDrive(double x , double y, double rx, double multiplier) {

         x = bot.driver.getLeftX() * multiplier;
         y = bot.driver.getLeftY() * multiplier;


            rx *= -bot.rotMultiplier;

            double botHeading = odo.getHeading(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1; // counteract imperfect strafe

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            double[] powers = {frontLeftPower, frontRightPower, backLeftPower, backRightPower};
            double[] normalizedPowers = normalizeWheelSpeeds(powers);

            frontLeft.setPower(normalizedPowers[0]);
            frontRight.setPower(normalizedPowers[1]);
            backLeft.setPower(normalizedPowers[2]);
            backRight.setPower(normalizedPowers[3]);
    }





    public void resetEncoders() {
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    private double[] normalizeWheelSpeeds(double[] speeds) {
        if (largestAbsolute(speeds) > 1) {
            double max = largestAbsolute(speeds);
            for (int i = 0; i < speeds.length; i++){
                speeds[i] /= max;
            }
        }
        return speeds;
    }

    private double largestAbsolute(double[] arr) {
        double largestAbsolute = 0;
        for (double d : arr) {
            double absoluteValue = Math.abs(d);
            if (absoluteValue > largestAbsolute) {
                largestAbsolute = absoluteValue;
            }
        }
        return largestAbsolute;
    }


}
