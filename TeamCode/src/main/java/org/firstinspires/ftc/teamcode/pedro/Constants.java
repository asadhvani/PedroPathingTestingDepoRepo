package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.196856535869467);
                Controller secondaryTranslationalForward = Controller.proportional(0.07273322990025292);
                Controller primaryTranslationalLateral = Controller.proportional(0.2740001724462586);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10123574229944428);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017417616007800535));
                c.brake.set(Controller.proportionalFeedforward(0.014804973606630455));

                c.headingFeedback.set(Controller.proportional(3.076021328608879));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.059967852934883194, 0.006895879082997328));

                c.linearBrakeCoefficients.set(Matrix.diag(0.025020762828693116, 0.017179270246996378));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-2.5613434418835803E-4, -2.4220556547799544E-5));

                c.maxAchievableForwardVelocity.set(59.08626883007241);
                c.maxAchievableStrafeVelocity.set(50.417336390150936);
                c.naturalForwardDeceleration.set(33.9928767674464);
                c.naturalStrafeDeceleration.set(53.171935969491095);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.9331147802157664);
        c.yPodOffset.set(-5.782494131974348);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("left_front_mtr");
        c.frontRightName.set("right_front_mtr");
        c.backLeftName.set("left_back_mtr");
        c.backRightName.set("right_back_mtr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}