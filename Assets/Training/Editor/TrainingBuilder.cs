using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Training.Editor
{
    /// <summary>
    /// Generates the training scene from code and builds the headless training player:
    ///   Unity -batchmode -nographics -projectPath . -executeMethod Training.Editor.TrainingBuilder.BuildFromCommandLine -out Builds/Throw/Throw.exe
    /// The scene holds only the physics settings (copied from the Playground's ConfigManager) and an ArenaSpawner;
    /// arenas are made at startup, as many as the -arenas argument asks for.
    /// </summary>
    public static class TrainingBuilder
    {
        public const string ScenePath = "Assets/Training/Throw.unity";

        [MenuItem("Training/Build Scene")]
        public static void BuildScene()
        {
            var playground = EditorSceneManager.OpenScene("Assets/Scenes/Playground.unity", OpenSceneMode.Single);
            var source = playground.GetRootGameObjects()
                .SelectMany(g => g.GetComponentsInChildren<ConfigManager>(true))
                .First(c => c.gameObject.name == "ConfigManager");
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);
            var config = new GameObject("ConfigManager");
            SceneManager.MoveGameObjectToScene(config, scene);
            EditorUtility.CopySerialized(source, config.AddComponent<ConfigManager>());

            var spawner = new GameObject("Arenas").AddComponent<ArenaSpawner>();
            SceneManager.MoveGameObjectToScene(spawner.gameObject, scene);
            spawner.armPrefab = Load("Assets/HumanoidModels/handArm.prefab");
            spawner.ballPrefab = Load("Assets/Prefabs/Projectile.prefab");
            spawner.targetPrefab = Load("Assets/Prefabs/Target.prefab");

            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorSceneManager.CloseScene(playground, true);
            Debug.Log($"[train] wrote {ScenePath}: {ThrowAgent.CountActions(spawner.armPrefab)} actions, solver {config.GetComponent<ConfigManager>().solverIterations}, {config.GetComponent<ConfigManager>().fixedUpdateFrequency} Hz");
        }

        static GameObject Load(string path) => AssetDatabase.LoadAssetAtPath<GameObject>(path) ?? throw new FileNotFoundException(path);

        public static void BuildFromCommandLine()
        {
            try
            {
                string output = Arg("-out") ?? "Builds/Throw/Throw.exe";
                BuildScene();
                PlayerSettings.runInBackground = true;
                PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
                PlayerSettings.visibleInBackground = false;
                PlayerSettings.usePlayerLog = true;
                var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
                {
                    scenes = new[] { ScenePath },
                    locationPathName = output,
                    target = BuildTarget.StandaloneWindows64,
                    options = BuildOptions.None,
                });
                Debug.Log($"[train] build {report.summary.result} -> {output} ({report.summary.totalSize / 1e6:0} MB, {report.summary.totalErrors} errors)");
                EditorApplication.Exit(report.summary.result == BuildResult.Succeeded ? 0 : 1);
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                EditorApplication.Exit(1);
            }
        }

        static string Arg(string name)
        {
            var args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, name);
            return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
        }
    }
}
