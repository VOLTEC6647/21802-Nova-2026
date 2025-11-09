
package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import lombok.Getter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Config
public class Vision extends SubsystemBase {

    private final Limelight3A camera;
    @Getter private LLResult result;

    public static double TURN_P = 0.05;
    private MultipleTelemetry telemetry;



    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = hardwareMap.get(Limelight3A.class, "limelight");

        pipeline(0);
        initializeCamera();

    }

    public void initializeCamera() {
        camera.setPollRateHz(100);
        camera.start();
    }
    public void pipeline(int switchPipeline){
        camera.pipelineSwitch(switchPipeline);
    }

    public double getTurnPower() {
        if (!result.isValid()) {
            return 0.0;
        }
        double tx = getTx();

        return -tx * TURN_P;
    }

    public double getTx() {
        if (result == null) {
            return 0;
        }
        return result.getTx();
    }

    public double getTy() {
        if (result == null) {
            return 0;
        }
        return result.getTy();
    }
    public Pose mt2(double heading) {
        camera.updateRobotOrientation(heading);

        return new Pose(
                result.getBotpose_MT2().getPosition().x,
                result.getBotpose_MT2().getPosition().y,
                result.getBotpose_MT2().getOrientation().getYaw(AngleUnit.RADIANS),
                FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }




    @Override
    public void periodic() {
        result = camera.getLatestResult();


        if (result != null) {

            Pose3D botpose = result.getBotpose();

            telemetry.addData("Turn Power", getTurnPower());
            telemetry.addData("Ty", getTy());
            telemetry.addData("Tx", getTx());


        }
    }
}
