package application;

import java.util.*;

public class Drone {

    private final int droneID;
    private final DroneState state;//composition has dronestate
    private Controller controller;

    private Vector3D externalFormationForce = new Vector3D(0,0,0);
    private Vector3D externalRepulsiveForce = new Vector3D(0,0,0);
    private Vector3D externalEnvironmentForce = new Vector3D(0,0,0);

    private List<Drone> neighbors = new ArrayList<>();

   
    public Drone(int id, double mass, double drag, Vector3D startPos) {
        this.droneID = id; 
        this.state = new DroneState(
            startPos, 
            new Vector3D(0, 0, 0),
            mass, 
            new Vector3D(0.02, 0.02, 0.04), 
            drag
        );
        this.controller = new Controller(6.0, 4.0, 0.2); 
    }

    public int getDroneID() { return droneID; }
    public DroneState getState() { return state; }
    public Controller getController() { return controller; }
    public void setNeighbors(List<Drone> n) { this.neighbors = n; }
    public List<Drone> getNeighbors() { return neighbors; }

    public void setControllerInputs(double thrustZ, Vector3D torqueBody) {
        state.thrustZ = thrustZ;
        state.torque  = torqueBody;
    }

    public void setFormationForce(Vector3D f) {
        this.externalFormationForce = f;
        state.formationForce = f; 
    }

    public void setRepulsiveForce(Vector3D f) {
        this.externalRepulsiveForce = f;
        state.repulsiveForce = f;
    }

    public void setEnvironmentForce(Vector3D f) {
        this.externalEnvironmentForce = f;
        state.environmentForce = f; 
    }

    public void integrate(double dt) {
        Vector3D Fg = state.getGravity().multiply(state.mass);
        Vector3D Fd = new Vector3D(-state.dragCoefficient * state.velocity.x, 
                                   -state.dragCoefficient * state.velocity.y, 
                                   -state.dragCoefficient * state.velocity.z);
        double tz = state.thrustZ;
        double[][] R = state.rotationMatrix;
        Vector3D Ft = new Vector3D(R[0][2]*tz, R[1][2]*tz, R[2][2]*tz);

        Vector3D F = new Vector3D(0,0,0);
        F.addInPlace(Fg); F.addInPlace(Ft); F.addInPlace(Fd);// f=fg+gt+fd+otherforces from diff module //netforce
        F.addInPlace(externalFormationForce); 
        F.addInPlace(externalRepulsiveForce); 
        F.addInPlace(externalEnvironmentForce);
//updates acceleration, velocity and position
        state.acceleration = F.divide(state.mass);//sets accel
        state.velocity.addInPlace(state.acceleration.multiply(dt));
        state.position.addInPlace(state.velocity.multiply(dt));
        
        updateRotationMatrixWithOmega(state.rotationMatrix, state.angularVelocity, dt);
    }

    private void updateRotationMatrixWithOmega(double[][] R, Vector3D omega, double dt) {
        Vector3D w = omega.multiply(dt);
        double angle = w.magnitude();
        if (angle == 0) return;// to break 1/0=infinity
        Vector3D k = w.divide(angle);
        double c = Math.cos(angle), s = Math.sin(angle), omc = 1 - c;
        double[][] Vr = new double[3][3];
        Vr[0][0] = c + k.x*k.x*omc; Vr[0][1] = k.x*k.y*omc - k.z*s; Vr[0][2] = k.x*k.z*omc + k.y*s;
        Vr[1][0] = k.y*k.x*omc + k.z*s; Vr[1][1] = c + k.y*k.y*omc; Vr[1][2] = k.y*k.z*omc - k.x*s;
        Vr[2][0] = k.z*k.x*omc - k.y*s; Vr[2][1] = k.z*k.y*omc + k.x*s; Vr[2][2] = c + k.z*k.z*omc;
        double[][] Rnew = new double[3][3];
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
                Rnew[i][j] = R[i][0]*Vr[0][j] + R[i][1]*Vr[1][j] + R[i][2]*Vr[2][j];
        for (int i=0;i<3;i++) System.arraycopy(Rnew[i], 0, R[i], 0, 3);
    }
}