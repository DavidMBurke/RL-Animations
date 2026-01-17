using Unity.MLAgents;

public static class MLAgentsBootstrap
{
    [UnityEngine.RuntimeInitializeOnLoadMethod(UnityEngine.RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void Apply()
    {
        // If this is disabled in-scene (or by another script), the Python trainer will hang at reset.
        Academy.Instance.AutomaticSteppingEnabled = true;
    }
}
