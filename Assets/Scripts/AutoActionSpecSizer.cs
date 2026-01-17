using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine;

/// <summary>
/// Keeps ML-Agents BehaviorParameters action spec in sync with StudentJointController.GetActionSize().
/// Attach to the same GameObject as BehaviorParameters (usually the Agent root).
/// </summary>
[DefaultExecutionOrder(-150)]
public class AutoActionSpecSizer : MonoBehaviour
{
    [Header("References")]
    public StudentJointController controller;
    public BehaviorParameters behaviorParameters;

    [Header("Editor")]
    [Tooltip("If true, also updates the BehaviorParameters while in edit mode (via OnValidate).")]
    public bool applyInEditor = true;

    [Tooltip("If true, attempts to rebuild controller bindings in edit mode before computing action size.")]
    public bool rebuildBindingsInEditor = true;

    [Header("Observations")]
    [Tooltip("If true, updates BehaviorParameters.VectorObservationSize to match ImitationAgent's CollectObservations output (12 + 8 * bindings).")]
    public bool autoSetVectorObservationSize = true;

    [Tooltip("Base observation floats (v,w,up,forward) = 4 Vector3 = 12.")]
    public int baseObservationFloats = 12;

    [Tooltip("Per binding observation floats (student quat + teacher quat) = 8.")]
    public int perBindingObservationFloats = 8;

    void Reset()
    {
        controller = GetComponent<StudentJointController>();
        behaviorParameters = GetComponent<BehaviorParameters>();
    }

    void Awake()
    {
        Apply(runtime: true);
    }

    void Start()
    {
        // If controller bindings weren't ready in Awake, retry once on Start.
        Apply(runtime: true);
    }

#if UNITY_EDITOR
    void OnValidate()
    {
        if (!applyInEditor) return;
        if (Application.isPlaying) return;
        Apply(runtime: false);
    }
#endif

    void Apply(bool runtime)
    {
        if (controller == null) controller = GetComponent<StudentJointController>();
        if (behaviorParameters == null) behaviorParameters = GetComponent<BehaviorParameters>();
        if (controller == null || behaviorParameters == null) return;

        if (runtime)
        {
            controller.EnsureInitialized(logIfEmpty: false);
        }

        if (autoSetVectorObservationSize)
        {
            int bindingCount = controller.bindings != null ? controller.bindings.Count : 0;
            int expectedObs = baseObservationFloats + perBindingObservationFloats * Mathf.Max(0, bindingCount);

            if (expectedObs > 0 && behaviorParameters.BrainParameters.VectorObservationSize != expectedObs)
            {
                behaviorParameters.BrainParameters.VectorObservationSize = expectedObs;

#if UNITY_EDITOR
                UnityEditor.EditorUtility.SetDirty(behaviorParameters);
#endif
            }
        }

#if UNITY_EDITOR
        if (!runtime && rebuildBindingsInEditor)
        {
            controller.RebuildBindingsSafely();
        }
#endif

        int size = controller.GetActionSize();
        if (size <= 0) return;

        var current = behaviorParameters.BrainParameters.ActionSpec;
        if (current.NumContinuousActions == size && current.NumDiscreteActions == 0) return;

        behaviorParameters.BrainParameters.ActionSpec = ActionSpec.MakeContinuous(size);

#if UNITY_EDITOR
        // Mark dirty so the change persists on prefabs/scenes.
        UnityEditor.EditorUtility.SetDirty(behaviorParameters);
#endif
    }
}
