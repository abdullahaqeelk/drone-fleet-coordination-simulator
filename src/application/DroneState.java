package application;

import application.Vector3D;

public class DroneState {

    //Translational
    public Vector3D position;
    public Vector3D velocity;
    public Vector3D acceleration;

    //Rotational
    public double[][] rotationMatrix;
    public Vector3D angularVelocity;
    public Vector3D angularAccel;

    //Physical params
    public double mass;
    public double dragCoefficient;
    public Vector3D inertia;

    //Inputs from controller
    public double thrustZ;
    public Vector3D torque;

    // ADDED (needed by Logger)
    public Vector3D formationForce = new Vector3D(0,0,0);
    public Vector3D repulsiveForce = new Vector3D(0,0,0);
    public Vector3D environmentForce = new Vector3D(0,0,0);

    public DroneState(Vector3D startPos, Vector3D startVel,
                      double mass, Vector3D inertia, double dragCoeff) {

        this.position = startPos;
        this.velocity = startVel;
        this.acceleration = new Vector3D(0,0,0);

        this.rotationMatrix = createIdentityMatrix();
        this.angularVelocity = new Vector3D(0,0,0);
        this.angularAccel = new Vector3D(0,0,0);

        this.mass = mass;
        this.inertia = inertia;
        this.dragCoefficient = dragCoeff;

        this.thrustZ = 0;
        this.torque = new Vector3D(0,0,0);
    }

    private double[][] createIdentityMatrix() {
        double[][] I = new double[3][3];
        I[0][0] = 1; I[1][1] = 1; I[2][2] = 1;
        return I;
    }

    public Vector3D getGravity() {
        return new Vector3D(0, 0, -9.81);
    }
}
