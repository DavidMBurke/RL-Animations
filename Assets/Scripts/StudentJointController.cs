using System;
using System.Collections.Generic;
using UnityEngine;

public class StudentJointController : MonoBehaviour
{
    [Serializable]
    public class JointBinding
    {
        public string boneName;                  // student bone name (matches teacher bone name)
        public ConfigurableJoint joint;          // joint on the student bone
        public Transform studentBone;            // cached
        public Transform teacherBone;            // cached

        public bool isHinge;                     // elbows/knees true
        public Vector3 hingeAxisLocal = Vector3.right; // axis in studentBone LOCAL space (tune if needed)

        // Cached local rotations at initialization
        public Quaternion studentStartLocalRot;
        public Quaternion teacherStartLocalRot;

        // For hinge: cached base target in local space
        public Quaternion studentBaseLocalRot;
    }

    [Header("References")]
    public Transform studentRoot;
    public TeacherPoseProvider teacher;

    [Header("Bindings")]
    public List<JointBinding> bindings = new();

    [Header("Action scaling")]
    [Tooltip("Max degrees for hinge offset actions (elbow/knee).")]
    public float hingeMaxOffsetDeg = 20f;

    [Tooltip("Max degrees for non-hinge per-axis offset actions (pitch/yaw/roll).")]
    public float ballMaxOffsetDeg = 10f;

    // Actions are supplied by the Agent each decision.
    private float[] _latestActions;

    private Dictionary<string, Transform> _studentBones;

    void Awake()
    {
        if (studentRoot == null) studentRoot = transform;
        BuildStudentBoneMap();
        ResolveBindings();
    }

    void BuildStudentBoneMap()
    {
        _studentBones = new Dictionary<string, Transform>(256);
        foreach (var t in studentRoot.GetComponentsInChildren<Transform>())
        {
            if (!_studentBones.ContainsKey(t.name))
                _studentBones.Add(t.name, t);
        }
    }

    void ResolveBindings()
    {
        if (teacher == null) throw new Exception("StudentJointController: teacher reference not set.");

        foreach (var b in bindings)
        {
            if (b.joint == null) throw new Exception($"Binding {b.boneName}: joint is null.");

            if (!_studentBones.TryGetValue(b.boneName, out b.studentBone))
                throw new Exception($"Student bone not found: {b.boneName}");

            if (!teacher.TryGetBone(b.boneName, out b.teacherBone))
                throw new Exception($"Teacher bone not found: {b.boneName}");

            b.studentStartLocalRot = b.studentBone.localRotation;
            b.teacherStartLocalRot = b.teacherBone.localRotation;

            // Base local rot used for hinge angle offsets
            b.studentBaseLocalRot = b.studentStartLocalRot;
        }
    }

    public int GetActionSize()
    {
        // Hinge: 1 float
        // Ball (3DOF): 3 floats (pitch/yaw/roll offsets)
        int count = 0;
        foreach (var b in bindings)
            count += b.isHinge ? 1 : 3;
        return count;
    }

    public void SetActions(float[] actions)
    {
        _latestActions = actions;
    }

    void FixedUpdate()
    {
        if (_latestActions == null) return;

        int idx = 0;
        foreach (var b in bindings)
        {
            // Reference pose from teacher (bone local rotation)
            Quaternion teacherLocal = b.teacherBone.localRotation;

            // Create offset from actions
            Quaternion offset;
            if (b.isHinge)
            {
                float a = Mathf.Clamp(_latestActions[idx++], -1f, 1f);
                float angle = a * hingeMaxOffsetDeg;

                // Hinge rotation around a local axis (you may need to adjust hingeAxisLocal per joint)
                offset = Quaternion.AngleAxis(angle, b.hingeAxisLocal);
            }
            else
            {
                float ax = Mathf.Clamp(_latestActions[idx++], -1f, 1f) * ballMaxOffsetDeg;
                float ay = Mathf.Clamp(_latestActions[idx++], -1f, 1f) * ballMaxOffsetDeg;
                float az = Mathf.Clamp(_latestActions[idx++], -1f, 1f) * ballMaxOffsetDeg;
                offset = Quaternion.Euler(ax, ay, az);
            }

            // Target local rotation: reference * offset
            Quaternion targetLocal = teacherLocal * offset;

            // Apply to joint.targetRotation (joint space conversion)
            JointTargetRotationUtil.SetTargetRotationLocal(b.joint, targetLocal, b.studentStartLocalRot);
        }
    }
}
