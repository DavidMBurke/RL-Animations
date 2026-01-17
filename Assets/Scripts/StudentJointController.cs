using System;
using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(-200)]
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

    [Header("Auto-detection")]
    [Tooltip("If true, automatically detect all ConfigurableJoints and create bindings on Awake.")]
    public bool autoDetectJoints = true;

    [Header("Bindings")]
    public List<JointBinding> bindings = new();

    [Header("Action scaling")]
    [Tooltip("Max degrees for hinge offset actions (elbow/knee).")]
    public float hingeMaxOffsetDeg = 20f;

    [Tooltip("Max degrees for non-hinge per-axis offset actions (pitch/yaw/roll).")]
    public float ballMaxOffsetDeg = 10f;

    [Header("Joint drive (recommended)")]
    [Tooltip("If true, configures ConfigurableJoint rotational drives at runtime so targetRotation actually actuates the ragdoll.")]
    public bool autoConfigureJointDrives = true;

    [Tooltip("Spring strength for the joint rotational drive (higher = stiffer tracking).")]
    public float jointDriveSpring = 1500f;

    [Tooltip("Damping for the joint rotational drive.")]
    public float jointDriveDamper = 80f;

    [Tooltip("Max force for the joint rotational drive.")]
    public float jointDriveMaxForce = 1000f;

    [Header("Debug")]
    [Tooltip("If true, ignores incoming actions and applies zero offsets (pure teacher tracking).")]
    public bool forceZeroActions = false;

    // Actions are supplied by the Agent each decision.
    private float[] _latestActions;

    private float[] _zeroActions;

    private Dictionary<string, Transform> _studentBones;

    private readonly List<(Rigidbody rb, Transform teacherBone)> _rbPoseBindings = new();

    private bool _initialized;

    void Awake()
    {
        if (studentRoot == null) studentRoot = transform;
        BuildStudentBoneMap();

        EnsureInitialized(logIfEmpty: true);

        if (autoConfigureJointDrives)
            ConfigureJointDrives();

        _zeroActions = new float[GetActionSize()];
    }

    void Start()
    {
        // Some scenes/components can finish wiring on Start; retry once.
        if (!_initialized)
        {
            EnsureInitialized(logIfEmpty: true);
            if (autoConfigureJointDrives)
                ConfigureJointDrives();
            _zeroActions = new float[GetActionSize()];
        }
    }

    void ConfigureJointDrives()
    {
        if (bindings == null) return;

        var drive = new JointDrive
        {
            positionSpring = jointDriveSpring,
            positionDamper = jointDriveDamper,
            maximumForce = jointDriveMaxForce
        };

        foreach (var b in bindings)
        {
            if (b == null || b.joint == null) continue;

            // Use slerp drive; it's generally the most stable for ragdoll pose tracking.
            b.joint.rotationDriveMode = RotationDriveMode.Slerp;
            b.joint.slerpDrive = drive;
        }
    }

#if UNITY_EDITOR
    void OnValidate()
    {
        // Keep zero-action buffer valid when bindings are edited in the inspector.
        if (Application.isPlaying) return;
        if (bindings == null) return;
        _zeroActions = new float[GetActionSize()];
    }
#endif

    [ContextMenu("Rebuild Bindings (AutoDetect)")]
    public void RebuildBindingsSafely()
    {
        if (studentRoot == null) studentRoot = transform;
        BuildStudentBoneMap();

        if (autoDetectJoints)
        {
            AutoDetectAndCreateBindings();
        }

        // Resolve will throw if teacher is missing; in edit mode we want to be tolerant.
        try
        {
            ResolveBindings();
        }
        catch (Exception)
        {
            // Ignore in editor; the user may still be wiring references.
        }

        BuildRigidbodyPoseBindings();

        _zeroActions = new float[GetActionSize()];
    }

    public bool EnsureInitialized(bool logIfEmpty)
    {
        if (studentRoot == null) studentRoot = transform;
        if (_studentBones == null || _studentBones.Count == 0) BuildStudentBoneMap();

        if (teacher == null)
        {
            if (logIfEmpty)
                Debug.LogWarning("StudentJointController: teacher reference not set; action size will be 0.");
            _initialized = false;
            return false;
        }

        if (autoDetectJoints && (bindings == null || bindings.Count == 0))
        {
            AutoDetectAndCreateBindings();
        }

        if (bindings == null || bindings.Count == 0)
        {
            if (logIfEmpty)
            {
                Debug.LogWarning(
                    "StudentJointController: no bindings were created. " +
                    "This makes action size 0. Check that your student ConfigurableJoint transform names match teacher bone names, " +
                    "and that TeacherPoseProvider.teacherRoot is set to the teacher skeleton root.");
            }
            _initialized = false;
            return false;
        }

        try
        {
            ResolveBindings();
            BuildRigidbodyPoseBindings();
            _initialized = true;
            return true;
        }
        catch (Exception e)
        {
            if (logIfEmpty)
                Debug.LogWarning($"StudentJointController: failed to resolve bindings: {e.Message}");
            _initialized = false;
            return false;
        }
    }

    void BuildStudentBoneMap()
    {
        _studentBones = new Dictionary<string, Transform>(256);
        foreach (var t in studentRoot.GetComponentsInChildren<Transform>(true))
        {
            if (!_studentBones.ContainsKey(t.name))
                _studentBones.Add(t.name, t);
        }
    }

    public bool TryGetStudentBone(string boneName, out Transform bone)
    {
        if (_studentBones == null || _studentBones.Count == 0)
        {
            if (studentRoot == null) studentRoot = transform;
            BuildStudentBoneMap();
        }

        return _studentBones.TryGetValue(boneName, out bone);
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

    void BuildRigidbodyPoseBindings()
    {
        _rbPoseBindings.Clear();
        if (teacher == null) return;

        foreach (var rb in studentRoot.GetComponentsInChildren<Rigidbody>())
        {
            if (teacher.TryGetBone(rb.transform.name, out var teacherBone))
            {
                _rbPoseBindings.Add((rb, teacherBone));
            }
        }
    }

    void AutoDetectAndCreateBindings()
    {
        if (teacher == null)
        {
            Debug.LogError("StudentJointController: teacher reference not set. Cannot auto-detect joints.");
            return;
        }

        bindings.Clear();

        // Find all ConfigurableJoints under student root
        var joints = studentRoot.GetComponentsInChildren<ConfigurableJoint>();
        Debug.Log($"Auto-detected {joints.Length} ConfigurableJoints on student model.");

        foreach (var joint in joints)
        {
            string boneName = joint.transform.name;

            // Check if teacher has matching bone
            if (!teacher.TryGetBone(boneName, out var teacherBone))
            {
                Debug.LogWarning($"Skipping joint on '{boneName}': no matching teacher bone found.");
                continue;
            }

            bool isHinge = IsHingeJoint(joint);
            Vector3 hingeAxis = GetHingeAxisForBone(boneName, joint);

            var binding = new JointBinding
            {
                boneName = boneName,
                joint = joint,
                studentBone = joint.transform,
                teacherBone = teacherBone,
                isHinge = isHinge,
                hingeAxisLocal = hingeAxis
            };

            bindings.Add(binding);
            Debug.Log($"Created binding for '{boneName}' (Hinge: {binding.isHinge}, Axis: {hingeAxis})");
        }
    }

    bool IsHingeJoint(ConfigurableJoint joint)
    {
        // A joint is considered a hinge if:
        // - AngularYMotion and AngularZMotion are locked
        // - AngularXMotion is free or limited
        bool yLocked = joint.angularYMotion == ConfigurableJointMotion.Locked;
        bool zLocked = joint.angularZMotion == ConfigurableJointMotion.Locked;
        bool xFree = joint.angularXMotion != ConfigurableJointMotion.Locked;

        return yLocked && zLocked && xFree;
    }

    Vector3 GetHingeAxisForBone(string boneName, ConfigurableJoint joint)
    {
        // Convert joint.axis from joint space to bone local space
        // Joint space is defined by the joint's starting rotation relative to the bone
        Quaternion jointToLocal = Quaternion.LookRotation(joint.axis, joint.secondaryAxis);
        Vector3 axisInBoneLocal = jointToLocal * Vector3.right; // Assuming hinge rotates around X in joint space

        // Common hinge joints with reasonable default axes (in bone local space)
        string lowerName = boneName.ToLower();
        
        if (lowerName.Contains("elbow"))
            return Vector3.forward; // Elbows typically bend forward/back
        
        if (lowerName.Contains("knee"))
            return Vector3.right; // Knees typically bend around right axis
        
        if (lowerName.Contains("shoulder") || lowerName.Contains("hip"))
            return axisInBoneLocal; // Use computed axis for ball joints treated as hinges
        
        // Default fallback
        return axisInBoneLocal;
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
        if (actions == null)
        {
            _latestActions = null;
            return;
        }

        int expected = GetActionSize();
        if (actions.Length != expected)
        {
            // Common failure mode: bindings weren't created early enough (expected==0).
            if (expected == 0)
            {
                EnsureInitialized(logIfEmpty: false);
                _zeroActions = new float[GetActionSize()];
                expected = GetActionSize();
            }

            if (actions.Length != expected)
            {
                Debug.LogWarning(
                    $"StudentJointController.SetActions: expected {expected} actions but got {actions.Length}. Ignoring. " +
                    "(If expected is 0, your bindings are empty and the agent cannot actuate the ragdoll.)");
                return;
            }
        }

        _latestActions = actions;
    }

    public void ClearActions()
    {
        _latestActions = null;
    }

    public void SnapToTeacherPose(bool resetVelocities = true)
    {
        // Align all rigidbodies that can be name-mapped (most ragdolls)
        foreach (var (rb, teacherBone) in _rbPoseBindings)
        {
            rb.position = teacherBone.position;
            rb.rotation = teacherBone.rotation;

            if (resetVelocities)
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                rb.Sleep();
            }
        }

        Physics.SyncTransforms();
    }

    void FixedUpdate()
    {
        // Allow validating pure teacher tracking even without an Agent.
        var actions = forceZeroActions ? _zeroActions : _latestActions;
        if (actions == null && !forceZeroActions) return;

        int idx = 0;
        foreach (var b in bindings)
        {
            // Reference pose from teacher (bone local rotation)
            Quaternion teacherLocal = b.teacherBone.localRotation;

            // Create offset from actions
            Quaternion offset;
            if (b.isHinge)
            {
                float a = Mathf.Clamp(actions[idx++], -1f, 1f);
                float angle = a * hingeMaxOffsetDeg;

                // Hinge rotation around a local axis (you may need to adjust hingeAxisLocal per joint)
                offset = Quaternion.AngleAxis(angle, b.hingeAxisLocal);
            }
            else
            {
                float ax = Mathf.Clamp(actions[idx++], -1f, 1f) * ballMaxOffsetDeg;
                float ay = Mathf.Clamp(actions[idx++], -1f, 1f) * ballMaxOffsetDeg;
                float az = Mathf.Clamp(actions[idx++], -1f, 1f) * ballMaxOffsetDeg;
                offset = Quaternion.Euler(ax, ay, az);
            }

            // Target local rotation: reference * offset
            Quaternion targetLocal = teacherLocal * offset;

            // Apply to joint.targetRotation (joint space conversion)
            JointTargetRotationUtil.SetTargetRotationLocal(b.joint, targetLocal, b.studentStartLocalRot);
        }
    }
}
