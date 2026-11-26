using System;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Showcase.Editor
{
    /// <summary>
    /// Records a rollout of a trained policy in the training scene, in batchmode play mode:
    ///   Unity -batchmode -nographics -projectPath . -executeMethod Showcase.Editor.RecordCommand.Run
    ///     -showcaseTake out.take [-showcaseSeconds 90] [-showcaseModel "Assets/Trained Models/X.onnx"]
    /// The model, when given, replaces the policy on every agent template for this run only (the scene is not saved),
    /// and the touch observations are switched on or off to match its input size (Clark's earlier runs saw 92 values,
    /// his later ones 96: the same 92 plus 4 touch and release timers).
    /// [-showcaseSeed N] seeds the ball and target spawns, so every policy recorded with N faces the same sequence.
    /// With no trainer connected the agents run inference. TakeRecorder writes the take and exits.
    /// </summary>
    public static class RecordCommand
    {
        public static void Run()
        {
            try
            {
                EditorSceneManager.OpenScene("Assets/Scenes/Playground.unity", OpenSceneMode.Single);
                foreach (var c in UnityEngine.Object.FindObjectsOfType<CameraWASDController>()) c.enabled = false; // reads the mouse, which batchmode lacks
                string modelPath = Arg("-showcaseModel");
                if (!string.IsNullOrEmpty(modelPath))
                {
                    var model = AssetDatabase.LoadMainAssetAtPath(modelPath)
                                ?? throw new ArgumentException($"no model at {modelPath}");
                    var templates = Resources.FindObjectsOfTypeAll<Unity.MLAgents.Policies.BehaviorParameters>()
                        .Where(b => b.gameObject.scene.IsValid());
                    foreach (var b in templates)
                    {
                        var so = new SerializedObject(b);
                        so.FindProperty("m_Model").objectReferenceValue = model;
                        so.ApplyModifiedPropertiesWithoutUndo();
                        Debug.Log($"[showcase] {b.gameObject.name} now runs {modelPath}");
                    }
                    MatchTouchObservations((Unity.Barracuda.NNModel)model);
                }
                if (string.IsNullOrEmpty(Arg("-showcaseTake"))) throw new ArgumentException("missing -showcaseTake");
                EditorApplication.isPlaying = true; // TakeRecorder attaches on scene load and exits the editor when done
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                EditorApplication.Exit(1);
            }
        }

        /// <summary>Sets ADD_PROJECTILE_TOUCH on the active ConfigManager to what the model's input size expects.</summary>
        static void MatchTouchObservations(Unity.Barracuda.NNModel nn)
        {
            var input = Unity.Barracuda.ModelLoader.Load(nn).inputs.First(i => i.name == "obs_0");
            int size = input.shape[input.shape.Length - 1];
            if (size != 92 && size != 96) throw new ArgumentException($"policy expects {size} observations; the Playground arm gives 92 or 96");
            foreach (var config in Resources.FindObjectsOfTypeAll<ConfigManager>().Where(c => c.gameObject.scene.IsValid() && c.gameObject.activeInHierarchy))
            {
                var so = new SerializedObject(config);
                so.FindProperty("ADD_PROJECTILE_TOUCH").boolValue = size == 96;
                so.ApplyModifiedPropertiesWithoutUndo();
                Debug.Log($"[showcase] {size} observations: touch data {(size == 96 ? "on" : "off")} on {config.gameObject.name}");
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
