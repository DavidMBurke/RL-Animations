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
    [SerializeField] Transform studentPelvis;
    [SerializeField] Transform studentHead;
    [SerializeField] Transform studentChest;
    [SerializeField] Transform studentSpine;
    [SerializeField] Transform studentUpperLegL;
    [SerializeField] Transform studentUpperLegR;
    [SerializeField] Transform studentClavicleL;
    [SerializeField] Transform studentClavicleR;
    Transform _teacherHandL, _teacherHandR, _teacherFootL, _teacherFootR;
    Transform _teacherPelvis, _teacherHead;
    Transform _teacherChest, _teacherSpine;

    [Header("Reward weights")]
    public float wRot = 0.7f;
    public float wPos = 0.25f;

    [Tooltip("Weight for root/reference-frame position tracking.")]
    public float wRootPos = 0.05f;

    [Tooltip("Weight for staying upright (helps standing/balance learn faster).")]
    public float wUpright = 0.15f;

    [Tooltip("Weight for matching reference-frame height (helps avoid collapsing).")]
    public float wHeight = 0.10f;

    [Tooltip("Small per-step reward to encourage staying alive (e.g. 0.001).")]
    public float aliveReward = 0.001f;

    [Header("Optional action penalties")]
    [Tooltip("Penalty scale for mean squared action magnitude.")]
    public float actionL2Penalty = 0.0f;
    [Tooltip("Penalty scale for mean squared action delta between steps.")]
    public float actionDeltaL2Penalty = 0.0f;

    [Tooltip("Penalty scale for squared reference-frame linear+angular velocity (useful for Idle imitation).")]
    public float motionL2Penalty = 0.0f;

    [Header("Reward sharpness")]
    public float kRot = 12f;   // higher = stricter tracking
    public float kPos = 25f;

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

    [Header("Episode length")]
    [Tooltip("If > 0, end the episode after this many seconds (physics time).")]
    public float maxEpisodeSeconds = 2.0f;

    [Header("Reset")]
    public bool snapStudentToTeacherOnReset = true;
    public bool randomizeTeacherPhaseOnReset = false;

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

    private bool _didLogFirstObservation;

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

        if (randomizeTeacherPhaseOnReset)
        {
            // Works when TeacherPoseProvider has an Animator reference + state name set.
            teacher.TrySetNormalizedTime(UnityEngine.Random.value);
        }

        if (snapStudentToTeacherOnReset)
        {
            controller.SnapToTeacherPose(resetVelocities: true);
        }
        else
        {
            ResetRagdollVelocities();
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
        posErr += EffErr(studentHandL, _teacherHandL, ref n);
        posErr += EffErr(studentHandR, _teacherHandR, ref n);
        posErr += EffErr(studentFootL, _teacherFootL, ref n);
        posErr += EffErr(studentFootR, _teacherFootR, ref n);

        posErr /= Mathf.Max(1, n);
        float rPos = Mathf.Exp(-kPos * posErr);

        // Reward: root/reference-frame absolute position alignment
        float rootErr = 0f;
        if (studentReferenceFrame != null)
        {
            if (teacherReferenceFrame != null)
            {
                // Express student reference position in teacher reference frame.
                Vector3 d = teacherReferenceFrame.InverseTransformPoint(studentReferenceFrame.position);
                rootErr = d.sqrMagnitude;
            }
            else
            {
                // Fallback: world-space distance to teacher root (if teacher exists) else 0.
                if (teacher != null)
                {
                    rootErr = (studentReferenceFrame.position - teacher.transform.position).sqrMagnitude;
                }
            }
        }

        float rRootPos = (kRootPos > 0f) ? Mathf.Exp(-kRootPos * rootErr) : 0f;

        // Reward: upright (encourages standing/balance)
        float upright01 = 0f;
        float uprightDot = 0f;
        float rUpright = 0f;
        if (studentPelvis != null && studentHead != null)
        {
            Vector3 upDir = (studentHead.position - studentPelvis.position);
            if (upDir.sqrMagnitude > 1e-6f)
            {
                upDir.Normalize();
                uprightDot = Vector3.Dot(upDir, Vector3.up);
            }
        }
        else
        {
            // Quadruped-friendly fallback: compute a torso "up" from bone positions.
            // lateral: right-left (prefer hips/upper legs, else clavicles)
            // forward: chest-spine (or head-spine)
            Vector3? left = (studentUpperLegL != null) ? studentUpperLegL.position : (studentClavicleL != null ? studentClavicleL.position : (Vector3?)null);
            Vector3? right = (studentUpperLegR != null) ? studentUpperLegR.position : (studentClavicleR != null ? studentClavicleR.position : (Vector3?)null);

            Vector3? spineP = (studentSpine != null) ? studentSpine.position : (Vector3?)null;
            Vector3? chestP = (studentChest != null) ? studentChest.position : (Vector3?)null;
            Vector3? headP = (studentHead != null) ? studentHead.position : (Vector3?)null;

            if (left.HasValue && right.HasValue && spineP.HasValue && (chestP.HasValue || headP.HasValue))
            {
                Vector3 lateral = (right.Value - left.Value);
                Vector3 forward = (chestP.HasValue ? (chestP.Value - spineP.Value) : (headP.Value - spineP.Value));

                if (lateral.sqrMagnitude > 1e-6f && forward.sqrMagnitude > 1e-6f)
                {
                    lateral.Normalize();
                    forward.Normalize();
                    Vector3 up = Vector3.Cross(lateral, forward);
                    if (up.sqrMagnitude > 1e-6f)
                    {
                        up.Normalize();
                        // Force the computed up to be the one closer to world up.
                        if (Vector3.Dot(up, Vector3.up) < 0f) up = -up;
                        uprightDot = Vector3.Dot(up, Vector3.up);
                    }
                }
            }

            // Last resort: use reference transform axis.
            if (uprightDot == 0f && studentReferenceFrame != null)
                uprightDot = Vector3.Dot(studentReferenceFrame.up, Vector3.up);
        }

        if (studentReferenceFrame != null)
        {
            upright01 = Mathf.Clamp01(0.5f * (uprightDot + 1f));
            // Penalize deviation from fully upright (uprightDot=1) using exp(-k * (1-u)^2)
            float uErr = (1f - upright01);
            rUpright = (kUpright > 0f) ? Mathf.Exp(-kUpright * uErr * uErr) : upright01;
        }

        // Reward: height (encourages not collapsing)
        float heightErr = 0f;
        float rHeight = 0f;
        if (studentPelvis != null)
        {
            float targetY = ( _teacherPelvis != null )
                ? _teacherPelvis.position.y
                : (teacherReferenceFrame != null ? teacherReferenceFrame.position.y : (teacher != null ? teacher.transform.position.y : studentPelvis.position.y));

            float dy = studentPelvis.position.y - targetY;
            heightErr = dy * dy;
            rHeight = (kHeight > 0f) ? Mathf.Exp(-kHeight * heightErr) : 0f;
        }
        else if (studentChest != null && studentSpine != null)
        {
            float torsoY = 0.5f * (studentChest.position.y + studentSpine.position.y);
            float targetY;
            if (_teacherChest != null && _teacherSpine != null)
                targetY = 0.5f * (_teacherChest.position.y + _teacherSpine.position.y);
            else if (teacherReferenceFrame != null)
                targetY = teacherReferenceFrame.position.y;
            else if (teacher != null)
                targetY = teacher.transform.position.y;
            else
                targetY = torsoY;

            float dy = torsoY - targetY;
            heightErr = dy * dy;
            rHeight = (kHeight > 0f) ? Mathf.Exp(-kHeight * heightErr) : 0f;
        }

        float total =
            wRot * rRot +
            wPos * rPos +
            wRootPos * rRootPos +
            wUpright * rUpright +
            wHeight * rHeight +
            aliveReward;

        if (actionL2Penalty > 0f)
        {
            float l2 = 0f;
            for (int i = 0; i < act.Length; i++) l2 += act[i] * act[i];
            l2 /= Mathf.Max(1, act.Length);
            total -= actionL2Penalty * l2;
        }

        if (actionDeltaL2Penalty > 0f)
        {
            float dl2 = 0f;
            int nAct = Mathf.Min(act.Length, _prevActions.Length);
            for (int i = 0; i < nAct; i++)
            {
                float d = act[i] - _prevActions[i];
                dl2 += d * d;
                _prevActions[i] = act[i];
            }
            dl2 /= Mathf.Max(1, nAct);
            total -= actionDeltaL2Penalty * dl2;
        }

        if (motionL2Penalty > 0f && studentReferenceFrame != null && studentReferenceRb != null)
        {
            Vector3 vLocal = studentReferenceFrame.InverseTransformDirection(studentReferenceRb.linearVelocity);
            Vector3 wLocal = studentReferenceFrame.InverseTransformDirection(studentReferenceRb.angularVelocity);
            float v2 = vLocal.sqrMagnitude;
            float w2 = wLocal.sqrMagnitude;
            total -= motionL2Penalty * (v2 + w2);
        }

        AddReward(total);

        // Stats for TensorBoard (helps diagnose plateaus)
        try
        {
            var stats = Academy.Instance.StatsRecorder;
            stats.Add("imitation/rRot", rRot);
            stats.Add("imitation/rPos", rPos);
            stats.Add("imitation/rRootPos", rRootPos);
            stats.Add("imitation/rUpright", rUpright);
            stats.Add("imitation/rHeight", rHeight);
            stats.Add("imitation/rotErr", rotErr);
            stats.Add("imitation/posErr", posErr);
            stats.Add("imitation/rootErr", rootErr);
            stats.Add("imitation/upright01", upright01);
            stats.Add("imitation/uprightDot", uprightDot);
            stats.Add("imitation/heightErr", heightErr);
            if (studentPelvis != null) stats.Add("imitation/pelvisY", studentPelvis.position.y);
        }
        catch
        {
            // Ignore (stats recorder may not be available in some contexts).
        }

        // Terminate if fallen (prefer pelvis height)
        Transform fallRef = studentPelvis != null ? studentPelvis : (studentSpine != null ? studentSpine : studentReferenceFrame);
        if (fallRef != null && fallRef.position.y < minPelvisHeight) EndEpisode();

        if (fallRef != null)
        {
            float upright = (studentPelvis != null && studentHead != null)
                ? uprightDot
                : Vector3.Dot(fallRef.up, Vector3.up);
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
        if (maxEpisodeSeconds <= 0f) return;
        if (StepCount <= 0) return;

        _episodeTimer += Time.fixedDeltaTime;
        if (_episodeTimer >= maxEpisodeSeconds)
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

        // Reference frame
        studentReferenceFrame = studentReferenceFrameOverride;
        if (studentReferenceFrame == null)
        {
            studentReferenceFrame = ResolveStudentBone(referenceBoneNameCandidates) ?? controller.studentRoot;
        }

        teacherReferenceFrame = teacherReferenceFrameOverride;
        if (teacherReferenceFrame == null)
        {
            teacherReferenceFrame = ResolveTeacherBone(referenceBoneNameCandidates) ?? teacher.transform;
        }

        // Reference RB
        studentReferenceRb = studentReferenceRigidbodyOverride;
        if (studentReferenceRb == null && studentReferenceFrame != null)
        {
            studentReferenceRb = studentReferenceFrame.GetComponent<Rigidbody>();
        }
        if (studentReferenceRb == null)
        {
            studentReferenceRb = controller.studentRoot.GetComponentInChildren<Rigidbody>();
        }

        // End effectors (student)
        studentHandL = ResolveStudentBone(handLeftNameCandidates);
        studentHandR = ResolveStudentBone(handRightNameCandidates);
        studentFootL = ResolveStudentBone(footLeftNameCandidates);
        studentFootR = ResolveStudentBone(footRightNameCandidates);

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

        // Upright/height bones (teacher)
        _teacherPelvis = ResolveTeacherBone(pelvisNameCandidates);
        _teacherHead = ResolveTeacherBone(headNameCandidates);

        // Quadruped torso (teacher)
        _teacherChest = ResolveTeacherBone(chestNameCandidates);
        _teacherSpine = ResolveTeacherBone(spineNameCandidates);
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
