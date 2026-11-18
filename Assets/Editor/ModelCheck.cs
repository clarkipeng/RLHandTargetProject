using System.Linq;
using Unity.Barracuda;
using UnityEditor;
using UnityEngine;

// Loads every trained ONNX policy with Barracuda and runs one forward pass on zeros, e.g.
//   tools/unity.sh -quit -executeMethod ModelCheck.Run -logFile Logs/models.log
public static class ModelCheck
{
    public static void Run()
    {
        int failed = 0;
        var paths = AssetDatabase.FindAssets("t:NNModel", new[] { "Assets/Trained Models" })
            .Select(AssetDatabase.GUIDToAssetPath).OrderBy(p => p).ToArray();
        foreach (var path in paths)
        {
            try
            {
                var model = ModelLoader.Load(AssetDatabase.LoadAssetAtPath<NNModel>(path));
                using var worker = WorkerFactory.CreateWorker(WorkerFactory.Type.CSharpBurst, model);
                var inputs = model.inputs.ToDictionary(
                    i => i.name,
                    i => new Tensor(i.shape.Select(d => d <= 0 ? 1 : d).ToArray()));
                worker.Execute(inputs);
                var outputs = string.Join(", ", model.outputs.Select(o => $"{o}{worker.PeekOutput(o).shape}"));
                foreach (var t in inputs.Values) t.Dispose();
                Debug.Log($"ModelCheck OK {path}: {outputs}");
            }
            catch (System.Exception e)
            {
                failed++;
                Debug.LogError($"ModelCheck FAIL {path}: {e.Message}");
            }
        }
        Debug.Log($"ModelCheck: {paths.Length - failed}/{paths.Length} models ran");
        EditorApplication.Exit(failed == 0 && paths.Length > 0 ? 0 : 1);
    }
}
