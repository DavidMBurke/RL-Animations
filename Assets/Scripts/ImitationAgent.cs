using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class ImitationAgent : Agent
{
    [Header("References")]
    public Transform studentPelvis;
    public Rigidbody studentPelvisRb;
    public TeacherPoseProvider teacher;
    public StudentJointController controller;

    [Header("End effectors (student + teacher names must match)")]
    public Transform studentHandL, studentHandR, studentFootL, studentFootR;
    public string teacherHandLName = "hand_L";
    public string teacherHandRName = "hand_R";
    public string teacherFootLName = "foot_L";
    public string teacherFootRName = "foot_R";

    private Transform _teacherHandL, _teacherHandR, _teacherFootL, _teacherFootR;

    [Header("Reward weights")]
    public float wRot = 0.7f;
    public float wPos = 0.25f;
    public float wAlive = 0.05f;

    [Header("Reward sharpness")]
    public float kRot = 12f;   // higher = stricter tracking
    public float kPos = 25f;

    [Header("Termination")]
    public float minPelvisHeight = 0.2f; // adjust for beam height

    private List<StudentJointController.JointBinding> _bindings;

    public override void Initialize()
    {
        if (studentPelvisRb == null) studentPelvisRb = studentPelvis.GetComponent<Rigidbody>();
        _bindings = controller.bindings;

        teacher.TryGetBone(teacherHandLName, out _teacherHandL);
        teacher.TryGetBone(teacherHandRName, out _teacherHandR);
        teacher.TryGetBone(teacherFootLName, out _teacherFootL);
        teacher.TryGetBone(teacherFootRName, out _teacherFootR);
    }

    public override void OnEpisodeBegin()
    {
        ResetRagdoll();
        // Optional: randomize teacher time/phase later for walking clips
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Root kinematics in pelvis local frame
        Vector3 v = studentPelvis.InverseTransformDirection(studentPelvisRb.linearVelocity);
        Vector3 w = studentPelvis.InverseTransformDirection(studentPelvisRb.angularVelocity);
        sensor.AddObservation(v);
        sensor.AddObservation(w);

        // Pelvis orientation (up + forward)
        sensor.AddObservation(studentPelvis.InverseTransformDirection(studentPelvis.up));
        sensor.AddObservation(studentPelvis.InverseTransformDirection(studentPelvis.forward));

        // Joint rotations: student + teacher (local)
        foreach (var b in _bindings)
        {
            Quaternion qs = b.studentBone.localRotation;
            Quaternion qt = b.teacherBone.localRotation;

            // Represent as quaternion components (x,y,z,w)
            sensor.AddObservation(qs.x); sensor.AddObservation(qs.y); sensor.AddObservation(qs.z); sensor.AddObservation(qs.w);
            sensor.AddObservation(qt.x); sensor.AddObservation(qt.y); sensor.AddObservation(qt.z); sensor.AddObservation(qt.w);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Send actions to controller
        var a = actions.ContinuousActions;
        float[] act = new float[a.Length];
        for (int i = 0; i < a.Length; i++) act[i] = a[i];
        controller.SetActions(act);

        // Reward: rotation alignment
        float rotErr = 0f;
        foreach (var b in _bindings)
        {
            float ang = Quaternion.Angle(b.studentBone.localRotation, b.teacherBone.localRotation) / 180f;
            rotErr += ang * ang;
        }
        rotErr /= Mathf.Max(1, _bindings.Count);
        float rRot = Mathf.Exp(-kRot * rotErr);

        // Reward: end-effector position alignment (pelvis-local)
        float posErr = 0f;
        int n = 0;
        posErr += EffErr(studentHandL, _teacherHandL); n++;
        posErr += EffErr(studentHandR, _teacherHandR); n++;
        posErr += EffErr(studentFootL, _teacherFootL); n++;
        posErr += EffErr(studentFootR, _teacherFootR); n++;

        posErr /= Mathf.Max(1, n);
        float rPos = Mathf.Exp(-kPos * posErr);

        float rAlive = 1f; // small constant as long as not terminated

        float total = wRot * rRot + wPos * rPos + wAlive * rAlive;
        AddReward(total);

        // Terminate if fallen
        if (studentPelvis.position.y < minPelvisHeight)
        {
            EndEpisode();
        }
    }

    float EffErr(Transform studentEff, Transform teacherEff)
    {
        if (studentEff == null || teacherEff == null) return 0f;
        Vector3 ps = studentPelvis.InverseTransformPoint(studentEff.position);
        Vector3 pt = studentPelvis.InverseTransformPoint(teacherEff.position);
        float d = (ps - pt).magnitude;     // meters
        return d * d;
    }

    void ResetRagdoll()
    {
        // Reset all rigidbodies under student root
        foreach (var rb in GetComponentsInChildren<Rigidbody>())
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            // If you have a known spawn pose/transform, set it here.
            // For now, just keep current positions/rotations.
        }
    }
}
