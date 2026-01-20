using System;
using System.Collections.Generic;
using System.Text;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class ImitationAgent : Agent
{
    [Header("References")]
    public TeacherPoseProvider teacher;
    public StudentJointController controller;

    [Header("Reference frame (auto-resolved)")]
    [Tooltip("Optional override for the student reference frame transform.")]
    public Transform studentReferenceFrameOverride;

    [Tooltip("Optional override for the reference rigidbody used for velocity observations.")]
    public Rigidbody studentReferenceRigidbodyOverride;

    [Tooltip("Optional override for the teacher reference frame transform (used for position error frame).")]
    public Transform teacherReferenceFrameOverride;

    [Tooltip("If true, chooses reference frames from rigidbodies/bindings instead of relying on specific bone names. Recommended when skeleton names change.")]
    public bool skeletonAgnosticReferenceFrames = true;

    [Tooltip("Bone name candidates (first match wins) used to find a reference frame when no pelvis exists.")]
    public string[] referenceBoneNameCandidates =
    {
        "chest", "Chest",
        "spine", "Spine", "spine_01", "Spine_01",
        "pelvis", "Pelvis", "hips", "Hips", "hip", "Hip",
        "root", "Root"
    };

    [Header("End effectors (student + teacher names must match)")]
    [Tooltip("Candidates for the LEFT hand bone name.")]
    public string[] handLeftNameCandidates = { "Hand.L", "Hand_L", "hand_L", "leftHand", "LeftHand", "L_Hand" };
    [Tooltip("Candidates for the RIGHT hand bone name.")]
    public string[] handRightNameCandidates = { "Hand.R", "Hand_R", "hand_R", "rightHand", "RightHand", "R_Hand" };
    [Tooltip("Candidates for the LEFT foot bone name.")]
    public string[] footLeftNameCandidates = { "Foot.L", "Foot_L", "foot_L", "leftFoot", "LeftFoot", "L_Foot" };
    [Tooltip("Candidates for the RIGHT foot bone name.")]
    public string[] footRightNameCandidates = { "Foot.R", "Foot_R", "foot_R", "rightFoot", "RightFoot", "R_Foot" };

    [Header("Additional effectors (improves arms/head alignment)")]
    [Tooltip("Candidates for the LEFT elbow/forearm bone name (used for extra end-effector position reward).")]
    public string[] elbowLeftNameCandidates = { "Elbow.L", "Elbow_L", "elbow_L", "LowerArm.L", "LowerArm_L", "Forearm.L", "Forearm_L" };
    [Tooltip("Candidates for the RIGHT elbow/forearm bone name (used for extra end-effector position reward).")]
    public string[] elbowRightNameCandidates = { "Elbow.R", "Elbow_R", "elbow_R", "LowerArm.R", "LowerArm_R", "Forearm.R", "Forearm_R" };

    [Header("Upright/height bones (recommended)")]
    [Tooltip("Candidates for the pelvis/hips bone name (used for fall detection + height reward).")]
    public string[] pelvisNameCandidates = { "Pelvis", "pelvis", "Hips", "hips", "Hip", "hip" };

    [Tooltip("Candidates for the head bone name (used for upright direction via pelvis->head).")]
    public string[] headNameCandidates = { "Head", "head" };

    [Tooltip("Candidates for torso bones used when there is no pelvis (e.g. quadrupeds).")]
    public string[] chestNameCandidates = { "Chest", "chest" };
    public string[] spineNameCandidates = { "Spine", "spine", "spine_01", "Spine_01" };
    public string[] upperLegLeftNameCandidates = { "UpperLeg.L", "UpperLeg_L", "UpperLegLeft", "UpperLegL" };
    public string[] upperLegRightNameCandidates = { "UpperLeg.R", "UpperLeg_R", "UpperLegRight", "UpperLegR" };
    public string[] clavicleLeftNameCandidates = { "Clavicle.L", "Clavicle_L", "ClavicleLeft", "ClavicleL" };
    public string[] clavicleRightNameCandidates = { "Clavicle.R", "Clavicle_R", "ClavicleRight", "ClavicleR" };

    [Header("Debug (resolved at runtime)")]
    [SerializeField] Transform studentReferenceFrame;
    [SerializeField] Rigidbody studentReferenceRb;
    [SerializeField] Transform teacherReferenceFrame;
    [SerializeField] Transform studentHandL;
    [SerializeField] Transform studentHandR;
    [SerializeField] Transform studentFootL;
    [SerializeField] Transform studentFootR;
    [SerializeField] Transform studentElbowL;
    [SerializeField] Transform studentElbowR;
    [SerializeField] Transform studentPelvis;
    [SerializeField] Transform studentHead;
    [SerializeField] Transform studentChest;
    [SerializeField] Transform studentSpine;
    [SerializeField] Transform studentUpperLegL;
    [SerializeField] Transform studentUpperLegR;
    [SerializeField] Transform studentClavicleL;
    [SerializeField] Transform studentClavicleR;
    Transform _teacherHandL, _teacherHandR, _teacherFootL, _teacherFootR;
    Transform _teacherElbowL, _teacherElbowR;
    Transform _teacherPelvis, _teacherHead;
    Transform _teacherChest, _teacherSpine;

    [Header("Reward weights")]
    public float wRot = 0.7f;
    public float wPos = 0.25f;

    [Tooltip("Weight for per-bone position tracking (all bound bones). This pushes full-skeleton alignment, not just end-effectors.")]
    public float wBonePos = 0.20f;

    [Tooltip("Weight for per-bone rotation tracking (all bound bones, frame-relative). This pushes full-skeleton rotation alignment.")]
    public float wBoneRot = 0.35f;

    [Header("Bone-local cost-only reward")]
    [Tooltip("Scale applied to the mean per-bone WORLD-position squared error.")]
    public float boneLocalPosCostScale = 5.0f;

    [Tooltip("Scale applied to the mean per-bone local-rotation squared error (radians^2).")]
    public float boneLocalRotCostScale = 1.0f;

    [Tooltip("Overall scale applied to -(posCost + rotCost). Increase if rewards are too close to 0.")]
    public float costToRewardScale = 1.0f;

    [Tooltip("Weight for root/reference-frame position tracking.")]
    public float wRootPos = 0.05f;

    [Tooltip("Weight for staying upright (helps standing/balance learn faster).")]
    public float wUpright = 0.15f;

    [Tooltip("Weight for matching reference-frame height (helps avoid collapsing).")]
    public float wHeight = 0.10f;

    [Header("Per-effector position weights")]
    [Tooltip("Relative weight for hand position tracking inside the end-effector position reward.")]
    public float posWeightHands = 2.0f;
    [Tooltip("Relative weight for foot position tracking inside the end-effector position reward.")]
    public float posWeightFeet = 1.0f;
    [Tooltip("Relative weight for head position tracking inside the end-effector position reward.")]
    public float posWeightHead = 1.5f;
    [Tooltip("Relative weight for elbow/forearm position tracking inside the end-effector position reward.")]
    public float posWeightElbows = 1.25f;

    [Header("Per-joint rotation weighting")]
    [Tooltip("Default weight for a joint when computing rotation error.")]
    public float rotWeightDefault = 1.0f;
    [Tooltip("Multiplier applied to arm joints (clavicle/shoulder/upperarm/lowerarm/hand).")]
    public float rotWeightArms = 2.0f;
    [Tooltip("Multiplier applied to head/neck joints.")]
    public float rotWeightHead = 2.0f;
    [Tooltip("Multiplier applied to torso joints (spine/chest).")]
    public float rotWeightTorso = 1.5f;

    [Header("Per-bone position weighting")]
    [Tooltip("Default weight for a bone when computing per-bone position error.")]
    public float bonePosWeightDefault = 1.0f;
    [Tooltip("Multiplier applied to arm bones for per-bone position error.")]
    public float bonePosWeightArms = 2.0f;
    [Tooltip("Multiplier applied to head/neck bones for per-bone position error.")]
    public float bonePosWeightHead = 6.0f;
    [Tooltip("Multiplier applied to torso bones for per-bone position error.")]
    public float bonePosWeightTorso = 1.5f;

    [Header("Bone-dominant preset")]
    [Tooltip("If true, ApplyBoneDominantPreset() will also reduce non-bone rewards (root/upright/height/alive) to near-zero.")]
    public bool boneDominantPresetSuppressShaping = true;

    [Tooltip("Small per-step reward to encourage staying alive (e.g. 0.001).")]
    public float aliveReward = 0.001f;

    [Header("Reward output")]
    [Tooltip("If true, converts costs into a bounded positive reward via exp(-cost). This is usually more stable for PPO than raw negative costs.")]
    public bool useExponentialReward = true;

    [Tooltip("Scale applied to the final reward added each step (after exp).")]
    public float rewardScale = 1.0f;

    [Tooltip("If > 0, adds a small positive reward each step to avoid flatlining when costs are near zero.")]
    public float aliveRewardPerStep = 0.0f;

    [Header("Optional action penalties")]
    [Tooltip("Penalty scale for mean squared action magnitude.")]
    public float actionL2Penalty = 0.0f;
    [Tooltip("Penalty scale for mean squared action delta between steps.")]
    public float actionDeltaL2Penalty = 0.0f;

    [Tooltip("Penalty scale for squared reference-frame linear+angular velocity (useful for Idle imitation).")]
    public float motionL2Penalty = 0.01f;

    [Tooltip("Relative weight for angular velocity inside the motion penalty. (rad/s)^2 is often much larger than (m/s)^2.")]
    public float motionAngularWeight = 0.05f;

    [Tooltip("Only apply motion penalty when pose cost is already below this threshold (prevents discouraging corrective motion when far from target).")]
    public float motionPoseGateCost = 0.06f;

    [Tooltip("Clamp the raw motion cost to avoid huge gradients during falls/collisions.")]
    public float motionCostClamp = 5.0f;

    [Header("Reward sharpness")]
    public float kRot = 12f;   // higher = stricter tracking
    public float kPos = 25f;

    [Tooltip("Sharpness for per-bone position tracking. Higher = stricter pose matching. Typical range: 20-200 depending on rig scale.")]
    public float kBonePos = 80f;

    [Tooltip("Sharpness for per-bone rotation tracking. Higher = stricter rotation matching.")]
    public float kBoneRot = 20f;

    [Tooltip("Sharpness for root/reference-frame position tracking.")]
    public float kRootPos = 10f;

    [Tooltip("Sharpness for upright reward. Higher makes tipping punished more aggressively.")]
    public float kUpright = 6f;

    [Tooltip("Sharpness for reference-frame height matching.")]
    public float kHeight = 10f;

    [Header("Termination")]
    public float minPelvisHeight = 0.2f; // adjust for beam height

    [Tooltip("Minimum upright dot (pelvis up · world up).")]
    public float minUprightDot = 0.5f;
    [Tooltip("End episode if tipped longer than this many seconds.")]
    public float maxTippedTime = 1.0f;

    private const float EpisodeSeconds = 5.0f;

    [Header("Reset")]
    public bool snapStudentToTeacherOnReset = true;
    public bool randomizeTeacherPhaseOnReset = false;

    [Tooltip("If true, explicitly sets the teacher to a known normalized time at the start of every episode (deterministic resets).")]
    public bool forceTeacherTimeOnReset = true;

    [Range(0f, 1f)]
    public float teacherNormalizedTimeOnReset = 0f;

    [Tooltip("If true, applies a random impulse on reset to force the policy to learn balance/standing.")]
    public bool applyRandomImpulseOnReset = true;

    [Tooltip("Impulse magnitude applied on reset (ForceMode.Impulse). Try 1-5 for small nudges, 5-15 for stronger pushes.")]
    public float resetImpulseStrength = 6f;

    [Tooltip("How vertical the reset impulse can be. 0 = purely horizontal, 1 = fully random sphere.")]
    [Range(0f, 1f)]
    public float resetImpulseVerticalMix = 0.1f;

    [Header("Debug logging")]
    [Tooltip("If true, logs a single snapshot of the observation values on the first CollectObservations call.")]
    public bool logFirstObservationSnapshot = true;

    [Tooltip("If true, logs the snapshot every episode (can be noisy).")]
    public bool logFirstObservationEachEpisode = false;

    private List<StudentJointController.JointBinding> _bindings;

    private float _tippedTimer;
    private float[] _prevActions;
    private float _episodeTimer;

    private Vector3 _rootOffsetInTeacherFrameOnReset;

    private bool _didLogFirstObservation;

    // Smoothly blend control authority in at the start of each episode.
    // This avoids violent initial corrections when the ragdoll spawns already near the target pose.
    private const float ActionRampSeconds = 0.50f;

    [ContextMenu("Apply Bone-Dominant Reward Preset")]
    public void ApplyBoneDominantPreset()
    {
        // Make bone-level accuracy dominate.
        wBonePos = 2.0f;
        wBoneRot = 2.0f;
        kBonePos = 120f;
        kBoneRot = 40f;

        // Put extra emphasis on the front half.
        bonePosWeightArms = 4.0f;
        bonePosWeightHead = 4.0f;
        bonePosWeightTorso = 2.0f;
        rotWeightArms = 4.0f;
        rotWeightHead = 4.0f;
        rotWeightTorso = 2.0f;

        // Keep end-effector tracking relevant but secondary.
        wPos = 0.10f;
        wRot = 0.10f;

        if (boneDominantPresetSuppressShaping)
        {
            wRootPos = 0.0f;
            wUpright = 0.02f;
            wHeight = 0.02f;
            aliveReward = 0.0f;
        }
    }

    public override void Initialize()
    {
        ResolveReferences();

        _didLogFirstObservation = false;

        if (controller != null)
        {
            _bindings = controller.bindings;
            _prevActions = new float[Mathf.Max(1, controller.GetActionSize())];
        }
        else
        {
            _bindings = new List<StudentJointController.JointBinding>();
            _prevActions = new float[1];
            Debug.LogError("ImitationAgent: Missing StudentJointController reference; observations/actions will be zeros.");
        }
    }

    public override void OnEpisodeBegin()
    {
        ResolveReferences();
        _tippedTimer = 0f;
        _episodeTimer = 0f;

        if (logFirstObservationEachEpisode)
            _didLogFirstObservation = false;

        // Ensure teacher pose is explicitly sampled at the episode boundary.
        // Without this, the teacher Animator can drift forward in real time between episodes,
        // making two runs with "the same settings" start from different poses.
        if (teacher != null)
        {
            if (randomizeTeacherPhaseOnReset)
                teacher.TrySetNormalizedTime(UnityEngine.Random.value);
            else if (forceTeacherTimeOnReset)
                teacher.TrySetNormalizedTime(teacherNormalizedTimeOnReset);
        }

        // IMPORTANT: clear any stale actions from the previous episode.
        // If the DecisionRequester is not every FixedUpdate, the joint controller can otherwise
        // apply the last episode's action for a few physics ticks, injecting energy.
        if (controller != null)
            controller.SetZeroActionsNow();

        if (snapStudentToTeacherOnReset)
        {
            controller.SnapToTeacherPose(resetVelocities: true);
        }
        else
        {
            ResetRagdollVelocities();
        }

        // Apply any updated joint drive tuning (spring/damper/maxForce) at a clean boundary.
        // This makes it easy to tweak values during Play Mode and see them take effect next episode.
        if (controller != null)
            controller.ApplyJointDriveSettingsIfDirty();

        // Cache initial root offset in the teacher reference frame so root tracking is robust
        // even when the teacher/ghost rig is placed elsewhere in the scene.
        if (teacherReferenceFrame != null && studentReferenceFrame != null)
        {
            _rootOffsetInTeacherFrameOnReset = teacherReferenceFrame.InverseTransformPoint(studentReferenceFrame.position);
        }
        else
        {
            _rootOffsetInTeacherFrameOnReset = Vector3.zero;
        }

        // Create a balancing problem: without disturbances the task can be too trivial and rewards flat-line.
        if (applyRandomImpulseOnReset && resetImpulseStrength > 0f && studentReferenceRb != null)
        {
            Vector3 dir = UnityEngine.Random.onUnitSphere;
            // Blend toward horizontal impulse.
            Vector3 horiz = new Vector3(dir.x, 0f, dir.z);
            if (horiz.sqrMagnitude < 1e-6f) horiz = Vector3.forward;
            horiz.Normalize();
            Vector3 mixed = Vector3.Slerp(horiz, dir.normalized, resetImpulseVerticalMix).normalized;
            studentReferenceRb.AddForce(mixed * resetImpulseStrength, ForceMode.Impulse);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (studentReferenceFrame == null || studentReferenceRb == null || _bindings == null)
            ResolveReferences();

        if (logFirstObservationSnapshot && !_didLogFirstObservation)
        {
            Debug.Log(BuildFirstObservationSnapshot());
            _didLogFirstObservation = true;
        }

        // Always emit a fixed-size observation vector:
        // base 12 floats + 8 floats per joint binding.

        // Root kinematics in reference local frame
        if (studentReferenceFrame != null && studentReferenceRb != null)
        {
            Vector3 v = studentReferenceFrame.InverseTransformDirection(studentReferenceRb.linearVelocity);
            Vector3 w = studentReferenceFrame.InverseTransformDirection(studentReferenceRb.angularVelocity);
            sensor.AddObservation(v);
            sensor.AddObservation(w);

            // World axes expressed in reference local frame (useful upright heading cues)
            sensor.AddObservation(studentReferenceFrame.InverseTransformDirection(Vector3.up));
            sensor.AddObservation(studentReferenceFrame.InverseTransformDirection(Vector3.forward));
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
        }

        // Joint rotations: student + teacher (local)
        if (_bindings == null)
            return;

        foreach (var b in _bindings)
        {
            Quaternion qs = Quaternion.identity;
            Quaternion qt = Quaternion.identity;

            if (b != null)
            {
                if (b.studentBone != null) qs = b.studentBone.localRotation;
                if (b.teacherBone != null) qt = b.teacherBone.localRotation;
            }

            // Represent as quaternion components (x,y,z,w)
            sensor.AddObservation(qs.x);
            sensor.AddObservation(qs.y);
            sensor.AddObservation(qs.z);
            sensor.AddObservation(qs.w);
            sensor.AddObservation(qt.x);
            sensor.AddObservation(qt.y);
            sensor.AddObservation(qt.z);
            sensor.AddObservation(qt.w);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // When BehaviorParameters is set to Default and no model is assigned, ML-Agents will
        // fall back to the HeuristicPolicy. Implementing this prevents the log spam:
        // "Heuristic method called but not implemented. Returning placeholder actions."
        var continuous = actionsOut.ContinuousActions;
        for (int i = 0; i < continuous.Length; i++)
            continuous[i] = 0f;

        var discrete = actionsOut.DiscreteActions;
        for (int i = 0; i < discrete.Length; i++)
            discrete[i] = 0;
    }

    string BuildFirstObservationSnapshot()
    {
        var sb = new StringBuilder(2048);
        int bindingCount = (_bindings != null) ? _bindings.Count : 0;
        int expectedObs = 12 + 8 * Mathf.Max(0, bindingCount);
        int actionSize = (controller != null) ? controller.GetActionSize() : -1;

        int sharedTransformCount = 0;
        bool sharedReferenceFrame = (studentReferenceFrame != null && teacherReferenceFrame != null && studentReferenceFrame == teacherReferenceFrame);

        sb.AppendLine("[ImitationAgent] First observation snapshot");
        sb.AppendLine($"- bindings: {bindingCount} (expected obs floats: {expectedObs})");
        sb.AppendLine($"- actionSize: {actionSize}");
        sb.AppendLine($"- studentReferenceFrame: {(studentReferenceFrame != null ? studentReferenceFrame.name : "<null>")}");
        sb.AppendLine($"- teacherReferenceFrame: {(teacherReferenceFrame != null ? teacherReferenceFrame.name : "<null>")}");
        sb.AppendLine($"- studentReferenceRb: {(studentReferenceRb != null ? studentReferenceRb.name : "<null>")}");
        if (sharedReferenceFrame)
            sb.AppendLine("- WARNING: studentReferenceFrame and teacherReferenceFrame are the SAME Transform instance.");

        if (studentReferenceFrame != null && studentReferenceRb != null)
        {
            Vector3 vLocal = studentReferenceFrame.InverseTransformDirection(studentReferenceRb.linearVelocity);
            Vector3 wLocal = studentReferenceFrame.InverseTransformDirection(studentReferenceRb.angularVelocity);
            Vector3 upLocal = studentReferenceFrame.InverseTransformDirection(Vector3.up);
            Vector3 fwdLocal = studentReferenceFrame.InverseTransformDirection(Vector3.forward);

            sb.AppendLine($"- vLocal: {vLocal}");
            sb.AppendLine($"- wLocal: {wLocal}");
            sb.AppendLine($"- upLocal: {upLocal}");
            sb.AppendLine($"- fwdLocal: {fwdLocal}");
            sb.AppendLine($"- refPos(world): {studentReferenceFrame.position}");
            sb.AppendLine($"- refRot(world): {studentReferenceFrame.rotation.eulerAngles}");
        }
        else
        {
            sb.AppendLine("- reference kinematics: <missing refs>");
        }

        if (_bindings == null)
        {
            sb.AppendLine("- bindings: <null>");
            return sb.ToString();
        }

        // Per-joint snapshot: student/teacher local rotations (quaternion components).
        for (int i = 0; i < _bindings.Count; i++)
        {
            var b = _bindings[i];
            if (b == null)
            {
                sb.AppendLine($"- [{i}] <null binding>");
                continue;
            }

            var studentName = (b.studentBone != null) ? b.studentBone.name : "<null>";
            var teacherName = (b.teacherBone != null) ? b.teacherBone.name : "<null>";

            if (b.studentBone != null && b.teacherBone != null && b.studentBone == b.teacherBone)
                sharedTransformCount++;

            Quaternion qs = (b.studentBone != null) ? b.studentBone.localRotation : Quaternion.identity;
            Quaternion qt = (b.teacherBone != null) ? b.teacherBone.localRotation : Quaternion.identity;

            float angDeg = 0f;
            if (b.studentBone != null && b.teacherBone != null)
                angDeg = Quaternion.Angle(qs, qt);

            sb.AppendLine($"- [{i}] {b.boneName} (student:{studentName}, teacher:{teacherName})");
            sb.AppendLine($"    qs(xyzw): ({qs.x:F4}, {qs.y:F4}, {qs.z:F4}, {qs.w:F4})");
            sb.AppendLine($"    qt(xyzw): ({qt.x:F4}, {qt.y:F4}, {qt.z:F4}, {qt.w:F4})");
            sb.AppendLine($"    angleDeg: {angDeg:F3}");
        }

        if (sharedTransformCount > 0)
        {
            sb.AppendLine($"- WARNING: {sharedTransformCount}/{bindingCount} bindings share the SAME Transform instance for student and teacher.");
            sb.AppendLine("  This usually means TeacherPoseProvider.teacherRoot is pointing at the STUDENT skeleton (or the teacher and student are the same object).");
            sb.AppendLine("  Fix: create a separate Teacher/Ghost skeleton GameObject (Animator-driven) and assign TeacherPoseProvider.teacherRoot to that object.");
        }

        if (sharedReferenceFrame || sharedTransformCount > 0)
        {
            sb.AppendLine("  When teacher and student share transforms, errors/rewards can flatline because both move together (rotErr/posErr/rootErr ~ 0). RL cannot learn.");
        }

        return sb.ToString();
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (studentReferenceFrame == null || _bindings == null)
            ResolveReferences();

        if (controller == null || _bindings == null)
        {
            // Can't act or compute meaningful rewards; keep the environment responsive.
            AddReward(0f);
            return;
        }

        controller.EnsureInitialized(logIfEmpty: false);

        // Send actions to controller
        var a = actions.ContinuousActions;
        float[] act = new float[a.Length];
        float actionScale = (ActionRampSeconds > 0f) ? Mathf.Clamp01(_episodeTimer / ActionRampSeconds) : 1f;
        for (int i = 0; i < a.Length; i++) act[i] = a[i] * actionScale;
        controller.SetActions(act);

        // COST (minimize):
        // 1) For each bound bone: squared distance between student and teacher bone positions expressed in each rig's reference frame.
        // 2) For each bound bone: squared deviation between student and teacher bone rotations relative to parent.
        // 3) Optional stillness term: penalize reference-frame linear+angular velocity (prevents gyration around a pose).
        // 4) Root/upright/height shaping: crucial to prevent collapse or in-place imitation for locomotion.
        float posCostWeightedSum = 0f;
        float posWeightSum = 0f;
        float rotCostWeightedSum = 0f;
        float rotWeightSum = 0f;
        float posMaxSqr = 0f;
        float rotMaxRad = 0f;
        int boneCount = 0;

        foreach (var b in _bindings)
        {
            if (b == null || b.studentBone == null || b.teacherBone == null) continue;

            // Position cost: compare bone positions in a stable reference frame.
            // IMPORTANT: comparing absolute world positions only works if teacher+student rigs are co-located.
            // In practice the teacher/ghost is often placed elsewhere in the scene, so we compare positions in
            // each rig's reference frame (pose-relative, but still derived from world positions).
            float posW = GetBonePosWeight(b.boneName);
            Vector3 ps = (studentReferenceFrame != null)
                ? studentReferenceFrame.InverseTransformPoint(b.studentBone.position)
                : b.studentBone.position;
            Vector3 pt = (teacherReferenceFrame != null)
                ? teacherReferenceFrame.InverseTransformPoint(b.teacherBone.position)
                : b.teacherBone.position;

            Vector3 ds = ps - pt;
            float posSqr = ds.sqrMagnitude;
            posCostWeightedSum += posW * posSqr;
            posWeightSum += posW;
            if (posSqr > posMaxSqr) posMaxSqr = posSqr;

            // Rotation cost: compare child rotation relative to its parent.
            Transform sp = b.studentBone.parent;
            Transform tp = b.teacherBone.parent;
            if (sp == null || tp == null) continue;

            float rotW = GetRotWeight(b.boneName);
            Quaternion qsRel = Quaternion.Inverse(sp.rotation) * b.studentBone.rotation;
            Quaternion qtRel = Quaternion.Inverse(tp.rotation) * b.teacherBone.rotation;
            float angRad = Quaternion.Angle(qsRel, qtRel) * Mathf.Deg2Rad;
            rotCostWeightedSum += rotW * (angRad * angRad);
            rotWeightSum += rotW;
            if (angRad > rotMaxRad) rotMaxRad = angRad;

            boneCount++;
        }

        float posCost = (posWeightSum > 0f) ? (posCostWeightedSum / posWeightSum) : 0f;
        float rotCost = (rotWeightSum > 0f) ? (rotCostWeightedSum / rotWeightSum) : 0f;

        // Root tracking: keep the student reference frame (pelvis/chest/root) aligned with the teacher's.
        // This is key for locomotion imitation (otherwise the agent can match pose in-place).
        float rootPosCost = 0f;
        if (teacherReferenceFrame != null && studentReferenceFrame != null)
        {
            Vector3 rootOffsetNow = teacherReferenceFrame.InverseTransformPoint(studentReferenceFrame.position);
            Vector3 delta = rootOffsetNow - _rootOffsetInTeacherFrameOnReset;
            rootPosCost = delta.sqrMagnitude; // meters^2
        }

        // Height tracking: match reference-frame height relative to teacher.
        float heightCost = 0f;
        if (teacherReferenceFrame != null)
        {
            Transform hRef = studentReferenceFrame;
            Transform tRef = teacherReferenceFrame;
            if (hRef != null && tRef != null)
            {
                float dy = (hRef.position.y - tRef.position.y);
                heightCost = dy * dy;
            }
        }

        // Upright: encourage staying oriented similarly to world-up.
        float uprightCost = 0f;
        {
            Transform uRef = studentReferenceFrame;
            if (uRef != null)
            {
                float uprightDot = Mathf.Clamp(Vector3.Dot(uRef.up, Vector3.up), -1f, 1f);
                // Convert to a smooth cost in [0, 4] (when dot=-1 => cost 4).
                float c = 1f - uprightDot;
                uprightCost = c * c;
            }
        }

        // Stillness penalty (in a stable frame). For Idle imitation this helps remove oscillation.
        float motionCost = 0f;
        float motionCostClamped = 0f;
        float motionGate = 0f;
        float motionLinSpeed = 0f;
        float motionAngSpeed = 0f;
        if (motionL2Penalty > 0f && studentReferenceRb != null)
        {
            Transform frame = teacherReferenceFrame != null ? teacherReferenceFrame : studentReferenceFrame;
            Vector3 v = studentReferenceRb.linearVelocity;
            Vector3 w = studentReferenceRb.angularVelocity;
            if (frame != null)
            {
                v = frame.InverseTransformDirection(v);
                w = frame.InverseTransformDirection(w);
            }

            motionLinSpeed = v.magnitude;
            motionAngSpeed = w.magnitude;
            motionCost = v.sqrMagnitude + motionAngularWeight * w.sqrMagnitude;

            // Gate stillness so it doesn't fight necessary corrective movement when the pose is bad.
            // Gate = 0 when far from target, approaches 1 as pose gets close.
            float poseCostRaw = posCost + rotCost;
            if (motionPoseGateCost > 1e-6f)
                motionGate = Mathf.Clamp01(1f - (poseCostRaw / motionPoseGateCost));
            else
                motionGate = 1f;

            // Clamp to avoid blow-ups from impacts/falls.
            motionCostClamped = Mathf.Min(motionCost, Mathf.Max(0f, motionCostClamp));
        }

        float totalCost =
            boneLocalPosCostScale * posCost +
            boneLocalRotCostScale * rotCost +
            motionL2Penalty * motionGate * motionCostClamped +
            wRootPos * rootPosCost +
            wHeight * heightCost +
            wUpright * uprightCost;

        float reward;
        if (useExponentialReward)
        {
            // Bounded in (0, 1]. Larger cost -> smaller reward.
            reward = Mathf.Exp(-costToRewardScale * totalCost) * rewardScale;
        }
        else
        {
            // Raw negative cost (kept for backwards compatibility).
            reward = -costToRewardScale * totalCost;
        }

        if (aliveRewardPerStep > 0f)
            reward += aliveRewardPerStep;

        AddReward(reward);

        // Human-interpretable diagnostics (Unity units are meters).
        float posRmsM = Mathf.Sqrt(Mathf.Max(0f, posCost));
        float posMaxM = Mathf.Sqrt(Mathf.Max(0f, posMaxSqr));
        float rotRmsRad = Mathf.Sqrt(Mathf.Max(0f, rotCost));
        float rotRmsDeg = rotRmsRad * Mathf.Rad2Deg;
        float rotMaxDeg = rotMaxRad * Mathf.Rad2Deg;

        // Stats for TensorBoard
        try
        {
            var stats = Academy.Instance.StatsRecorder;
            stats.Add("imitation/costLocalPos", posCost);
            stats.Add("imitation/costLocalRot", rotCost);
            stats.Add("imitation/costTotal", totalCost);
            stats.Add("imitation/reward", reward);
            stats.Add("imitation/boneCount", boneCount);
            stats.Add("imitation/actionScale", actionScale);
            stats.Add("imitation/costWorldPos", posCost);
            stats.Add("imitation/costParentRelRot", rotCost);
            stats.Add("imitation/costMotion", motionCost);
            stats.Add("imitation/costMotionClamped", motionCostClamped);
            stats.Add("imitation/motionGate", motionGate);
            stats.Add("imitation/motionLinSpeed", motionLinSpeed);
            stats.Add("imitation/motionAngSpeed", motionAngSpeed);
            stats.Add("imitation/costRootPos", rootPosCost);
            stats.Add("imitation/costHeight", heightCost);
            stats.Add("imitation/costUpright", uprightCost);
            stats.Add("imitation/costLocalPosRmsCm", posRmsM * 100f);
            stats.Add("imitation/costLocalPosMaxCm", posMaxM * 100f);
            stats.Add("imitation/costLocalRotRmsDeg", rotRmsDeg);
            stats.Add("imitation/costLocalRotMaxDeg", rotMaxDeg);
        }
        catch
        {
        }

        // Terminate if fallen (prefer pelvis height)
        Transform fallRef = studentReferenceFrame;
        if (fallRef != null && fallRef.position.y < minPelvisHeight) EndEpisode();

        if (fallRef != null)
        {
            float upright = Vector3.Dot(fallRef.up, Vector3.up);
            if (upright < minUprightDot)
            {
                _tippedTimer += Time.fixedDeltaTime;
                if (_tippedTimer >= maxTippedTime) EndEpisode();
            }
            else
            {
                _tippedTimer = 0f;
            }
        }
    }

    void FixedUpdate()
    {
        if (StepCount <= 0) return;

        _episodeTimer += Time.fixedDeltaTime;
        if (_episodeTimer >= EpisodeSeconds)
        {
            EndEpisode();
        }
    }

    float EffErr(Transform studentEff, Transform teacherEff, ref int count)
    {
        if (studentEff == null || teacherEff == null) return 0f;

        // IMPORTANT: compute in a stable frame (teacher frame preferred), not in the student's own frame.
        // Using the student's reference frame can hide global misalignment (e.g., collapsing/rotating) and inflate reward.
        Transform frame = teacherReferenceFrame != null ? teacherReferenceFrame : null;

        Vector3 ps;
        Vector3 pt;
        if (frame != null)
        {
            ps = frame.InverseTransformPoint(studentEff.position);
            pt = frame.InverseTransformPoint(teacherEff.position);
        }
        else
        {
            ps = studentEff.position;
            pt = teacherEff.position;
        }
        float d = (ps - pt).magnitude;     // meters
        count++;
        return d * d;
    }

    float EffErrWeighted(Transform studentEff, Transform teacherEff, float weight, ref float weightedErrSum, ref float weightSum)
    {
        if (studentEff == null || teacherEff == null) return 0f;
        if (weight <= 0f) return 0f;

        // IMPORTANT: compute in a stable frame (teacher frame preferred), not in the student's own frame.
        Transform frame = teacherReferenceFrame != null ? teacherReferenceFrame : null;

        Vector3 ps;
        Vector3 pt;
        if (frame != null)
        {
            ps = frame.InverseTransformPoint(studentEff.position);
            pt = frame.InverseTransformPoint(teacherEff.position);
        }
        else
        {
            ps = studentEff.position;
            pt = teacherEff.position;
        }

        float d = (ps - pt).magnitude;
        float err = d * d;
        weightedErrSum += weight * err;
        weightSum += weight;
        return err;
    }

    float GetRotWeight(string boneName)
    {
        if (string.IsNullOrEmpty(boneName)) return rotWeightDefault;
        string n = boneName.ToLowerInvariant();

        if (n.Contains("spine") || n.Contains("chest"))
            return rotWeightDefault * rotWeightTorso;

        if (n.Contains("head") || n.Contains("neck"))
            return rotWeightDefault * rotWeightHead;

        if (n.Contains("clav") || n.Contains("should") || n.Contains("upperarm") || n.Contains("lowerarm") || n.Contains("forearm") || n.Contains("elbow") || n.Contains("hand") || n.Contains("arm"))
            return rotWeightDefault * rotWeightArms;

        return rotWeightDefault;
    }

    float GetBonePosWeight(string boneName)
    {
        if (string.IsNullOrEmpty(boneName)) return bonePosWeightDefault;
        string n = boneName.ToLowerInvariant();

        if (n.Contains("spine") || n.Contains("chest"))
            return bonePosWeightDefault * bonePosWeightTorso;

        if (n.Contains("head") || n.Contains("neck"))
            return bonePosWeightDefault * bonePosWeightHead;

        if (n.Contains("clav") || n.Contains("should") || n.Contains("upperarm") || n.Contains("lowerarm") || n.Contains("forearm") || n.Contains("elbow") || n.Contains("hand") || n.Contains("arm"))
            return bonePosWeightDefault * bonePosWeightArms;

        return bonePosWeightDefault;
    }

    void ResetRagdollVelocities()
    {
        // Reset all rigidbodies under student root
        foreach (var rb in controller.studentRoot.GetComponentsInChildren<Rigidbody>())
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }

    void ResolveReferences()
    {
        if (controller == null) controller = GetComponent<StudentJointController>();
        if (teacher == null) teacher = FindFirstObjectByType<TeacherPoseProvider>();
        if (controller == null || teacher == null) return;

        // Ensure bindings exist (useful if Agent initializes before controller auto-detect).
        if (controller.bindings == null || controller.bindings.Count == 0)
        {
            controller.RebuildBindingsSafely();
        }

        // Reference frame + RB
        studentReferenceFrame = studentReferenceFrameOverride;
        studentReferenceRb = studentReferenceRigidbodyOverride;

        if (skeletonAgnosticReferenceFrames)
        {
            if (studentReferenceRb == null)
                studentReferenceRb = controller.studentRoot.GetComponent<Rigidbody>() ?? controller.studentRoot.GetComponentInChildren<Rigidbody>();

            if (studentReferenceFrame == null)
                studentReferenceFrame = (studentReferenceRb != null) ? studentReferenceRb.transform : controller.studentRoot;

            teacherReferenceFrame = teacherReferenceFrameOverride;
            if (teacherReferenceFrame == null)
            {
                // Prefer mapping the chosen student reference bone by name if possible; otherwise use teacher root.
                if (teacher.TryGetBone(studentReferenceFrame.name, out var mapped))
                    teacherReferenceFrame = mapped;
                else
                    teacherReferenceFrame = teacher.transform;
            }
        }
        else
        {
            studentReferenceFrame = studentReferenceFrameOverride;
            if (studentReferenceFrame == null)
                studentReferenceFrame = ResolveStudentBone(referenceBoneNameCandidates) ?? controller.studentRoot;

            teacherReferenceFrame = teacherReferenceFrameOverride;
            if (teacherReferenceFrame == null)
                teacherReferenceFrame = ResolveTeacherBone(referenceBoneNameCandidates) ?? teacher.transform;

            if (studentReferenceRb == null && studentReferenceFrame != null)
                studentReferenceRb = studentReferenceFrame.GetComponent<Rigidbody>();
            if (studentReferenceRb == null)
                studentReferenceRb = controller.studentRoot.GetComponentInChildren<Rigidbody>();
        }

        // End-effectors and special bones are optional. If your skeleton names change, the
        // per-bone binding-based imitation reward still works without these.
        if (!skeletonAgnosticReferenceFrames)
        {
            // End effectors (student)
            studentHandL = ResolveStudentBone(handLeftNameCandidates);
            studentHandR = ResolveStudentBone(handRightNameCandidates);
            studentFootL = ResolveStudentBone(footLeftNameCandidates);
            studentFootR = ResolveStudentBone(footRightNameCandidates);
            studentElbowL = ResolveStudentBone(elbowLeftNameCandidates);
            studentElbowR = ResolveStudentBone(elbowRightNameCandidates);

            // Upright/height bones (student)
            studentPelvis = ResolveStudentBone(pelvisNameCandidates);
            studentHead = ResolveStudentBone(headNameCandidates);

            // Quadruped torso/axes bones (student)
            studentChest = ResolveStudentBone(chestNameCandidates);
            studentSpine = ResolveStudentBone(spineNameCandidates);
            studentUpperLegL = ResolveStudentBone(upperLegLeftNameCandidates);
            studentUpperLegR = ResolveStudentBone(upperLegRightNameCandidates);
            studentClavicleL = ResolveStudentBone(clavicleLeftNameCandidates);
            studentClavicleR = ResolveStudentBone(clavicleRightNameCandidates);

            // End effectors (teacher)
            _teacherHandL = ResolveTeacherBone(handLeftNameCandidates);
            _teacherHandR = ResolveTeacherBone(handRightNameCandidates);
            _teacherFootL = ResolveTeacherBone(footLeftNameCandidates);
            _teacherFootR = ResolveTeacherBone(footRightNameCandidates);
            _teacherElbowL = ResolveTeacherBone(elbowLeftNameCandidates);
            _teacherElbowR = ResolveTeacherBone(elbowRightNameCandidates);

            // Upright/height bones (teacher)
            _teacherPelvis = ResolveTeacherBone(pelvisNameCandidates);
            _teacherHead = ResolveTeacherBone(headNameCandidates);

            // Quadruped torso (teacher)
            _teacherChest = ResolveTeacherBone(chestNameCandidates);
            _teacherSpine = ResolveTeacherBone(spineNameCandidates);
        }
    }

    Transform ResolveStudentBone(string[] candidates)
    {
        if (candidates == null || candidates.Length == 0) return null;
        foreach (var name in candidates)
        {
            if (string.IsNullOrWhiteSpace(name)) continue;
            if (controller.TryGetStudentBone(name, out var t)) return t;
        }
        return null;
    }

    Transform ResolveTeacherBone(string[] candidates)
    {
        if (candidates == null || candidates.Length == 0) return null;
        foreach (var name in candidates)
        {
            if (string.IsNullOrWhiteSpace(name)) continue;
            if (teacher.TryGetBone(name, out var t)) return t;
        }
        return null;
    }
}
