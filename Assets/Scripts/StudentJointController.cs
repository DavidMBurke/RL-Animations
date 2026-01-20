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

    [Tooltip("If true, applies bone-name-based hinge axis overrides (elbow/knee/etc). In most rigs this hurts; prefer using the computed axis from the joint.")]
    public bool useBoneNameHingeAxisOverrides = false;

    [Header("Teacher mapping")]
    [Tooltip("If true, applies the teacher's local rotation *delta from its bind pose* onto the student's bind pose. This is recommended when teacher and student rigs have different rest orientations (common cause of arms drifting/backwards).")]
    public bool applyTeacherDeltaFromBindPose = true;

    [Header("Joint drive (recommended)")]
    [Tooltip("If true, configures ConfigurableJoint rotational drives at runtime so targetRotation actually actuates the ragdoll.")]
    public bool autoConfigureJointDrives = true;

    [Tooltip("Spring strength for the joint rotational drive (higher = stiffer tracking).")]
    public float jointDriveSpring = 1500f;

    [Tooltip("Damping for the joint rotational drive.")]
    public float jointDriveDamper = 80f;

    [Tooltip("Max force for the joint rotational drive.")]
    public float jointDriveMaxForce = 1000f;

    [Header("Drive ramp (recommended)")]
    [Tooltip("Ramps joint drive strength from 0 to 1 over this many seconds after a reset. Helps prevent 'springing' energy when the ragdoll teleports to a new pose.")]
    public float driveRampSeconds = 0.25f;

    [Header("Reset")]
    [Tooltip("If true, SnapToTeacherPose will temporarily set all ragdoll rigidbodies kinematic and disable collisions while teleporting to avoid explosive impulses carrying across resets.")]
    public bool hardSnapDisablesCollisions = true;

    [Tooltip("If true, performs the hard snap over two phases across a FixedUpdate (teleport while kinematic + collisions off, then restore next physics tick). This helps prevent contact solver explosions from persisting across resets.")]
    public bool hardSnapTwoPhase = true;

    [Tooltip("If true, SnapToTeacherPose will zero velocities for ALL rigidbodies under the student root (not just those name-mapped to the teacher).")]
    public bool snapZeroesAllRigidbodies = true;

    [Tooltip("If true, snaps rigidbody poses using the joint bindings (student bone -> teacher bone) instead of relying on name-mapped rigidbodies. This is more reliable for ragdolls where only some bones have rigidbodies.")]
    public bool snapUsingBindings = true;

    [Tooltip("If true, automatically reapplies joint drive settings when values change (checked on episode begin via ImitationAgent).")]
    public bool reapplyDrivesWhenChanged = true;

    [Tooltip("If true, changing drive values in the Inspector during Play Mode will immediately reapply them.")]
    public bool reapplyDrivesOnValidateInPlayMode = false;

    private float _lastAppliedSpring = float.NaN;
    private float _lastAppliedDamper = float.NaN;
    private float _lastAppliedMaxForce = float.NaN;

    private float _driveRampTimer;
    private float _lastAppliedDriveScale = float.NaN;

    private Coroutine _hardSnapRoutine;

    struct JointState
    {
        public ConfigurableJoint joint;
        public ConfigurableJointMotion xMotion, yMotion, zMotion;
        public ConfigurableJointMotion angularXMotion, angularYMotion, angularZMotion;
        public RotationDriveMode rotationDriveMode;
        public JointDrive slerpDrive;
        public JointDrive angularXDrive;
        public JointDrive angularYZDrive;
        public bool enablePreprocessing;
        public JointProjectionMode projectionMode;
        public float projectionDistance;
        public float projectionAngle;
    }

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
            ApplyJointDriveSettingsNow(force: true);

        _zeroActions = new float[GetActionSize()];
    }

    void Start()
    {
        // Some scenes/components can finish wiring on Start; retry once.
        if (!_initialized)
        {
            EnsureInitialized(logIfEmpty: true);
            if (autoConfigureJointDrives)
                ApplyJointDriveSettingsNow(force: true);
            _zeroActions = new float[GetActionSize()];
        }
    }

    [ContextMenu("Apply Joint Drive Settings Now")]
    public void ApplyJointDriveSettingsNow(bool force = false)
    {
        if (!autoConfigureJointDrives) return;
        if (bindings == null) return;

        if (!force && !reapplyDrivesWhenChanged)
            return;

        if (!force &&
            Mathf.Approximately(_lastAppliedSpring, jointDriveSpring) &&
            Mathf.Approximately(_lastAppliedDamper, jointDriveDamper) &&
            Mathf.Approximately(_lastAppliedMaxForce, jointDriveMaxForce))
        {
            return;
        }

        ApplyJointDriveSettingsScaled(1.0f);

        _lastAppliedSpring = jointDriveSpring;
        _lastAppliedDamper = jointDriveDamper;
        _lastAppliedMaxForce = jointDriveMaxForce;
        _lastAppliedDriveScale = 1.0f;
    }

    void ApplyJointDriveSettingsScaled(float scale)
    {
        if (bindings == null) return;

        float s = Mathf.Max(0f, scale);
        var drive = new JointDrive
        {
            positionSpring = jointDriveSpring * s,
            positionDamper = jointDriveDamper * s,
            maximumForce = jointDriveMaxForce * s
        };

        foreach (var b in bindings)
        {
            if (b == null || b.joint == null) continue;

            // Use slerp drive; it's generally the most stable for ragdoll pose tracking.
            b.joint.rotationDriveMode = RotationDriveMode.Slerp;
            b.joint.slerpDrive = drive;
        }
    }

    public void ApplyJointDriveSettingsIfDirty()
    {
        ApplyJointDriveSettingsNow(force: false);
    }

    /// <summary>
    /// Call on episode begin / reset to prevent joints from applying a large impulse.
    /// It (1) resets the drive ramp and (2) sets joint targets to match the CURRENT pose.
    /// </summary>
    public void NotifyEpisodeResetBoundary()
    {
        _driveRampTimer = 0f;
        _lastAppliedDriveScale = float.NaN;
        MatchJointTargetsToCurrentPose();

        // Start with drives at 0 (or small) and ramp up.
        if (autoConfigureJointDrives)
            ApplyJointDriveSettingsScaled(0f);
    }

    void MatchJointTargetsToCurrentPose()
    {
        if (bindings == null) return;
        foreach (var b in bindings)
        {
            if (b == null || b.joint == null || b.studentBone == null) continue;
            JointTargetRotationUtil.SetTargetRotationLocal(b.joint, b.studentBone.localRotation, b.studentStartLocalRot);
        }
    }

#if UNITY_EDITOR
    void OnValidate()
    {
        // Keep zero-action buffer valid when bindings are edited in the inspector.
        if (Application.isPlaying)
        {
            if (reapplyDrivesOnValidateInPlayMode)
                ApplyJointDriveSettingsNow(force: true);
            return;
        }
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

        if (!useBoneNameHingeAxisOverrides)
            return axisInBoneLocal;

        // Optional: bone-name heuristics (only if explicitly enabled)
        string lowerName = boneName.ToLower();

        if (lowerName.Contains("elbow"))
            return Vector3.forward;

        if (lowerName.Contains("knee"))
            return Vector3.right;

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

    public void SetZeroActionsNow()
    {
        // Critical for clean episode resets: prevents reusing the previous episode's last action
        // for one or more physics ticks (DecisionRequester period > 1), which can inject energy.
        if (_zeroActions == null || _zeroActions.Length != GetActionSize())
            _zeroActions = new float[GetActionSize()];
        _latestActions = _zeroActions;
    }

    public void SnapToTeacherPose(bool resetVelocities = true)
    {
        if (studentRoot == null) studentRoot = transform;

        if (hardSnapDisablesCollisions && hardSnapTwoPhase && Application.isPlaying)
        {
            if (_hardSnapRoutine != null) StopCoroutine(_hardSnapRoutine);
            _hardSnapRoutine = StartCoroutine(HardSnapCoroutine(resetVelocities));
            return;
        }

        // Optional hard snap: disable collisions + make kinematic during teleport.
        // This prevents an already-exploding ragdoll (deep interpenetration, high velocity) from immediately exploding again after reset.
        Rigidbody[] allRbs = studentRoot.GetComponentsInChildren<Rigidbody>();
        var jointStates = CaptureAndFreeAllJoints();
        bool[] prevKinematic = null;
        bool[] prevDetect = null;

        if (hardSnapDisablesCollisions)
        {
            prevKinematic = new bool[allRbs.Length];
            prevDetect = new bool[allRbs.Length];

            for (int i = 0; i < allRbs.Length; i++)
            {
                var rb = allRbs[i];
                prevKinematic[i] = rb.isKinematic;
                prevDetect[i] = rb.detectCollisions;
                rb.detectCollisions = false;
                rb.isKinematic = true;
            }
        }

        // Align rigidbodies to the teacher pose.
        ApplyPoseSnap();

        // Make sure the transform hierarchy reflects teleported rigidbodies before touching joint targets.
        Physics.SyncTransforms();

        // Ensure joints are not trying to pull toward a stale target.
        NotifyEpisodeResetBoundary();

        if (resetVelocities)
        {
            if (snapZeroesAllRigidbodies)
            {
                foreach (var rb in allRbs)
                {
                    rb.linearVelocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
            }
            else
            {
                ZeroVelocitiesForSnappedBodies();
            }
        }

        Physics.SyncTransforms();

        if (hardSnapDisablesCollisions)
        {
            for (int i = 0; i < allRbs.Length; i++)
            {
                var rb = allRbs[i];
                rb.isKinematic = prevKinematic[i];
                rb.detectCollisions = prevDetect[i];
            }
        }

        RestoreAllJoints(jointStates);

        if (resetVelocities)
        {
            foreach (var rb in allRbs)
                rb.Sleep();
        }

        Physics.SyncTransforms();
    }

    void ApplyPoseSnap()
    {
        if (teacher == null) return;

        // Snap ALL rigidbodies we can map, using both:
        // - joint bindings (accurate for driven bones)
        // - name-mapped rigidbodies (covers extra bodies that may not have joints/bindings)
        var seen = new HashSet<Rigidbody>();

        if (snapUsingBindings && bindings != null && bindings.Count > 0)
        {
            foreach (var b in bindings)
            {
                if (b == null || b.studentBone == null || b.teacherBone == null) continue;
                var rb = b.studentBone.GetComponent<Rigidbody>();
                if (rb == null) continue;
                if (!seen.Add(rb)) continue;

                rb.position = b.teacherBone.position;
                rb.rotation = b.teacherBone.rotation;
            }
        }

        foreach (var (rb, teacherBone) in _rbPoseBindings)
        {
            if (rb == null || teacherBone == null) continue;
            if (!seen.Add(rb)) continue;
            rb.position = teacherBone.position;
            rb.rotation = teacherBone.rotation;
        }
    }

    void ZeroVelocitiesForSnappedBodies()
    {
        if (snapUsingBindings && bindings != null && bindings.Count > 0)
        {
            var seen = new HashSet<Rigidbody>();
            foreach (var b in bindings)
            {
                if (b == null || b.studentBone == null) continue;
                var rb = b.studentBone.GetComponent<Rigidbody>();
                if (rb == null) continue;
                if (!seen.Add(rb)) continue;
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
            return;
        }

        foreach (var (rb, _) in _rbPoseBindings)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }

    System.Collections.IEnumerator HardSnapCoroutine(bool resetVelocities)
    {
        if (studentRoot == null) studentRoot = transform;

        var allRbs = studentRoot.GetComponentsInChildren<Rigidbody>();
        var jointStates = CaptureAndFreeAllJoints();
        var prevKinematic = new bool[allRbs.Length];
        var prevDetect = new bool[allRbs.Length];

        for (int i = 0; i < allRbs.Length; i++)
        {
            var rb = allRbs[i];
            prevKinematic[i] = rb.isKinematic;
            prevDetect[i] = rb.detectCollisions;
            rb.detectCollisions = false;
            rb.isKinematic = true;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        ApplyPoseSnap();
        Physics.SyncTransforms();

    // Now that transforms are correct, align joint targets to the current pose and ramp drives.
    NotifyEpisodeResetBoundary();

        // Let the physics engine observe the teleported pose with collisions disabled.
        yield return new WaitForFixedUpdate();

        for (int i = 0; i < allRbs.Length; i++)
        {
            var rb = allRbs[i];
            rb.isKinematic = prevKinematic[i];
            rb.detectCollisions = prevDetect[i];
            if (resetVelocities)
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                rb.Sleep();
            }
        }

        RestoreAllJoints(jointStates);
        Physics.SyncTransforms();
        _hardSnapRoutine = null;
    }

    List<JointState> CaptureAndFreeAllJoints()
    {
        var joints = studentRoot.GetComponentsInChildren<ConfigurableJoint>();
        var states = new List<JointState>(joints.Length);

        // Zero drive used while joints are temporarily freed.
        var zeroDrive = new JointDrive { positionSpring = 0f, positionDamper = 0f, maximumForce = 0f };

        foreach (var j in joints)
        {
            if (j == null) continue;

            states.Add(new JointState
            {
                joint = j,
                xMotion = j.xMotion,
                yMotion = j.yMotion,
                zMotion = j.zMotion,
                angularXMotion = j.angularXMotion,
                angularYMotion = j.angularYMotion,
                angularZMotion = j.angularZMotion,
                rotationDriveMode = j.rotationDriveMode,
                slerpDrive = j.slerpDrive,
                angularXDrive = j.angularXDrive,
                angularYZDrive = j.angularYZDrive,
                enablePreprocessing = j.enablePreprocessing,
                projectionMode = j.projectionMode,
                projectionDistance = j.projectionDistance,
                projectionAngle = j.projectionAngle
            });

            // Free all constraints during teleport.
            j.xMotion = ConfigurableJointMotion.Free;
            j.yMotion = ConfigurableJointMotion.Free;
            j.zMotion = ConfigurableJointMotion.Free;
            j.angularXMotion = ConfigurableJointMotion.Free;
            j.angularYMotion = ConfigurableJointMotion.Free;
            j.angularZMotion = ConfigurableJointMotion.Free;

            // Remove any drive impulses.
            j.rotationDriveMode = RotationDriveMode.Slerp;
            j.slerpDrive = zeroDrive;
            j.angularXDrive = zeroDrive;
            j.angularYZDrive = zeroDrive;

            // Avoid joint preprocessing/projection pushing bodies during the teleport frame.
            j.enablePreprocessing = false;
            j.projectionMode = JointProjectionMode.None;
        }

        return states;
    }

    void RestoreAllJoints(List<JointState> states)
    {
        if (states == null) return;
        foreach (var s in states)
        {
            if (s.joint == null) continue;

            s.joint.xMotion = s.xMotion;
            s.joint.yMotion = s.yMotion;
            s.joint.zMotion = s.zMotion;
            s.joint.angularXMotion = s.angularXMotion;
            s.joint.angularYMotion = s.angularYMotion;
            s.joint.angularZMotion = s.angularZMotion;
            s.joint.rotationDriveMode = s.rotationDriveMode;
            s.joint.slerpDrive = s.slerpDrive;
            s.joint.angularXDrive = s.angularXDrive;
            s.joint.angularYZDrive = s.angularYZDrive;
            s.joint.enablePreprocessing = s.enablePreprocessing;
            s.joint.projectionMode = s.projectionMode;
            s.joint.projectionDistance = s.projectionDistance;
            s.joint.projectionAngle = s.projectionAngle;
        }
    }

    void Update()
    {
        // Drive ramp uses scaled drives. Update() runs even when no Agent decisions happen.
        if (!autoConfigureJointDrives) return;
        if (driveRampSeconds <= 1e-6f) return;

        // Ramp timer in real time; FixedUpdate may not run if time scale is 0.
        _driveRampTimer += Time.deltaTime;
    }

    void LateUpdate()
    {
        // Apply ramp scaling at most once per frame.
        if (!autoConfigureJointDrives) return;
        if (driveRampSeconds <= 1e-6f) return;

        float t = Mathf.Clamp01(_driveRampTimer / driveRampSeconds);
        if (!float.IsNaN(_lastAppliedDriveScale) && Mathf.Abs(_lastAppliedDriveScale - t) < 0.02f)
            return;

        ApplyJointDriveSettingsScaled(t);
        _lastAppliedDriveScale = t;
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

            // Map teacher pose onto student bind pose.
            // Teacher: R_t = R_t0 * Δ  => Δ = inv(R_t0) * R_t
            // Student target: R_s = R_s0 * Δ
            Quaternion mappedTeacherLocal = teacherLocal;
            if (applyTeacherDeltaFromBindPose)
            {
                Quaternion delta = Quaternion.Inverse(b.teacherStartLocalRot) * teacherLocal;
                mappedTeacherLocal = b.studentStartLocalRot * delta;
            }

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
            Quaternion targetLocal = mappedTeacherLocal * offset;

            // Apply to joint.targetRotation (joint space conversion)
            JointTargetRotationUtil.SetTargetRotationLocal(b.joint, targetLocal, b.studentStartLocalRot);
        }
    }
}
