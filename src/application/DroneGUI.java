package application;

import javafx.application.Application;
import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.PerspectiveCamera;
import javafx.scene.PointLight;
import javafx.scene.SceneAntialiasing;
import javafx.scene.control.Label;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import javafx.animation.AnimationTimer;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DroneGUI extends Application {

    private Simulator simulator;
    private List<Drone> drones;
    private final Map<Integer, Group> droneVisuals = new HashMap<>();
    private final Map<Integer, Group> trails = new HashMap<>();

    // Visualization Scales and Bounds
    private final double SCALE = 20.0;      // 1 meter = 20 pixels
    private final double GRID_SIZE = 25;    // +- 25 meters
    private final double AXIS_LENGTH = GRID_SIZE * SCALE * 1.5;

    @Override
    public void start(Stage stage) {

        simulator = Main.getSimulator();
        drones = simulator.getDrones();

        // World (3D) root
        Group worldRoot = new Group();

        // 1) Build the 3D Environment (Graph Paper, Axes, Lighting)
        buildLighting(worldRoot);
        buildGraphPaper(worldRoot);
        buildAxes(worldRoot);

        // 2) Create Visuals for Drones and Trails
        for (Drone d : drones) {
            Group droneGroup = createDroneModel(d.getDroneID());

            // Trail group to hold spray particles
            Group trailGroup = new Group();
            trails.put(d.getDroneID(), trailGroup);
            worldRoot.getChildren().add(trailGroup);

            droneVisuals.put(d.getDroneID(), droneGroup);
            worldRoot.getChildren().add(droneGroup);
        }

        // HUD (time + Δt) - overlay label
        Label hud = new Label("t = 0.00 s   Δt = " + String.format("%.3f", Main.DT) + " s");
        hud.setTextFill(Color.WHITESMOKE);
        hud.setStyle("-fx-background-color: rgba(0,0,0,0.45); -fx-padding: 8; -fx-background-radius: 8;");
        hud.setTranslateX(15);
        hud.setTranslateY(15);

        Group overlayRoot = new Group(hud);

        // Root that contains BOTH world and overlay
        Group root = new Group(worldRoot, overlayRoot);

        // 3) Setup Camera
        PerspectiveCamera camera = new PerspectiveCamera(true);
        camera.setNearClip(0.1);
        camera.setFarClip(10000.0);

        camera.getTransforms().addAll(
                new Translate(0, -500, -800),
                new Rotate(-35, Rotate.X_AXIS)
        );

        // Scene Setup
        Scene scene = new Scene(root, 1200, 800, true, SceneAntialiasing.BALANCED);
        scene.setFill(Color.rgb(30, 30, 30));
        scene.setCamera(camera);

        stage.setTitle("CEP Pesticide Spraying Mission (3D View)");
        stage.setScene(scene);
        stage.show();

        // 4) Animation Loop
        AnimationTimer timer = new AnimationTimer() {
            private long lastUpdate = 0;
            private int frameCount = 0;

            @Override
            public void handle(long now) {
                if (now - lastUpdate >= 16_000_000) { // ~60 FPS

                    // Simulation time based on Δt
                    double tSec = frameCount * Main.DT;

                    // Update mission phases (GO -> SPRAY 10s -> RETURN), then step physics
                    Main.updateMission(tSec, drones);
                    simulator.step();

                    // Update visuals
                    updateVisuals(frameCount);

                    // Update HUD
                    hud.setText(String.format("t = %.2f s     Δt = %.3f s", tSec, Main.DT));

                    lastUpdate = now;
                    frameCount++;
                }
            }
        };
        timer.start();
    }

    // --- VISUALIZATION HELPERS ---

    private void updateVisuals(int frameCount) {
        for (Drone d : drones) {
            Vector3D p = d.getState().position;
            Group droneGroup = droneVisuals.get(d.getDroneID());

            // COORDINATE MAPPING: Physics Z (Altitude) -> JavaFX -Y (Up)
            double visualX = p.x * SCALE;
            double visualY = -p.z * SCALE;
            double visualZ = p.y * SCALE;

            droneGroup.setTranslateX(visualX);
            droneGroup.setTranslateY(visualY);
            droneGroup.setTranslateZ(visualZ);

            // IMPORTANT:
            // - No dots at start: spray particles ONLY during SPRAYING phase
            // - "Raining drops": add a few droplets frequently, slightly below drone, with small random spread
            if (Main.isSpraying(d.getDroneID()) && (frameCount % 2 == 0)) {
                addSprayParticle(d.getDroneID(), visualX, visualY, visualZ);
            }
        }
    }

    private void addSprayParticle(int droneID, double x, double y, double z) {

        // Small sphere = droplet
        Sphere particle = new Sphere(1.2);

        PhongMaterial material = new PhongMaterial(Color.web("#00ff99", 0.75));
        material.setSpecularColor(Color.WHITE);
        particle.setMaterial(material);

        // Random spread (looks like rain cone)
        double rx = (Math.random() - 0.5) * 18;  // sideways spread
        double rz = (Math.random() - 0.5) * 18;

        // Drop below drone (JavaFX +Y is down; remember we mapped altitude to -Y)
        double dropDown = 20 + Math.random() * 25;

        particle.setTranslateX(x + rx);
        particle.setTranslateY(y + dropDown);
        particle.setTranslateZ(z + rz);

        trails.get(droneID).getChildren().add(particle);

        // Cleanup old particles (keeps performance)
        if (trails.get(droneID).getChildren().size() > 600) {
            trails.get(droneID).getChildren().remove(0, 30);
        }
    }

    private Group createDroneModel(int id) {
        Group d = new Group();

        // Body
        Box body = new Box(10, 5, 10);
        PhongMaterial bodyMat = new PhongMaterial(Color.CRIMSON);
        bodyMat.setSpecularColor(Color.WHITE);
        body.setMaterial(bodyMat);

        // Arms
        PhongMaterial armMat = new PhongMaterial(Color.SILVER);
        armMat.setSpecularColor(Color.LIGHTGRAY);

        Box arm1 = new Box(30, 2, 2);
        arm1.setMaterial(armMat);

        Box arm2 = new Box(2, 2, 30);
        arm2.setMaterial(armMat);

        d.getChildren().addAll(arm1, arm2, body);
        return d;
    }

    // --- ENVIRONMENT VISUALIZATION METHODS ---

    private void buildLighting(Group root) {
        AmbientLight ambient = new AmbientLight(Color.rgb(80, 80, 80));

        PointLight sun = new PointLight(Color.WHITE);
        sun.setTranslateX(AXIS_LENGTH * 2);
        sun.setTranslateY(-AXIS_LENGTH * 1.5);
        sun.setTranslateZ(-AXIS_LENGTH * 2);

        root.getChildren().addAll(ambient, sun);
    }

    private void buildGraphPaper(Group root) {
        Color gridColor = Color.rgb(80, 80, 80);

        Box groundPlane = new Box(GRID_SIZE * 2 * SCALE, 1, GRID_SIZE * 2 * SCALE);
        groundPlane.setMaterial(new PhongMaterial(Color.web("#455A64")));
        groundPlane.setTranslateY(0.5);
        root.getChildren().add(groundPlane);

        for (double i = -GRID_SIZE; i <= GRID_SIZE; i += 2.0) {

            Cylinder gridLineX = new Cylinder(0.5, GRID_SIZE * 2 * SCALE);
            gridLineX.setMaterial(new PhongMaterial(gridColor));
            gridLineX.setRotationAxis(Rotate.X_AXIS);
            gridLineX.setRotate(90);
            gridLineX.setTranslateX(i * SCALE);
            gridLineX.setTranslateY(0);

            Cylinder gridLineZ = new Cylinder(0.5, GRID_SIZE * 2 * SCALE);
            gridLineZ.setMaterial(new PhongMaterial(gridColor));
            gridLineZ.setRotationAxis(Rotate.Z_AXIS);
            gridLineZ.setRotate(90);
            gridLineZ.setTranslateZ(i * SCALE);
            gridLineZ.setTranslateY(0);

            root.getChildren().addAll(gridLineX, gridLineZ);
        }
    }

    private void buildAxes(Group root) {
        double thickness = 2;

        Cylinder xAxis = new Cylinder(thickness, AXIS_LENGTH);
        xAxis.setMaterial(new PhongMaterial(Color.RED));
        xAxis.setRotationAxis(Rotate.Z_AXIS);
        xAxis.setRotate(90);
        xAxis.setTranslateX(AXIS_LENGTH / 2);

        Cylinder yAxis = new Cylinder(thickness, AXIS_LENGTH);
        yAxis.setMaterial(new PhongMaterial(Color.GREEN));
        yAxis.setTranslateY(-AXIS_LENGTH / 2);

        Cylinder zAxis = new Cylinder(thickness, AXIS_LENGTH);
        zAxis.setMaterial(new PhongMaterial(Color.BLUE));
        zAxis.setRotationAxis(Rotate.X_AXIS);
        zAxis.setRotate(90);
        zAxis.setTranslateZ(AXIS_LENGTH / 2);

        root.getChildren().addAll(xAxis, yAxis, zAxis);
    }

    public static void main(String[] args) {
        launch(args);
    }
}
