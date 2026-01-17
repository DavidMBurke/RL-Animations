using UnityEngine;

public static class HeadlessBootstrap
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void Apply()
    {
        Application.runInBackground = true;

        // Multi-instance builds can behave badly with exclusive fullscreen or vsync.
        Screen.fullScreen = false;
        QualitySettings.vSyncCount = 0;

        // Let ML-Agents control pacing (or run as fast as possible).
        Application.targetFrameRate = -1;
    }
}
