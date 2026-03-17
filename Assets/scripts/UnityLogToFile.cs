// 测试时把此组件挂到任意 GameObject（如 ROSConnection 或 Drone），
// 会在 Play 时把 Console 输出同时写入文件，便于检查通讯与 subscribe 是否成功。
using UnityEngine;
using System.IO;

public class UnityLogToFile : MonoBehaviour
{
    [Tooltip("Log file name under persistentDataPath (e.g. on Windows: AppData/LocalLow/...)")]
    public string logFileName = "drone_ros_test.log";
    [Tooltip("Only log when playing")]
    public bool onlyWhenPlaying = true;

    private StreamWriter _writer;
    private string _path;

    void OnEnable()
    {
        if (onlyWhenPlaying && !Application.isPlaying) return;
        _path = Path.Combine(Application.persistentDataPath, logFileName);
        try
        {
            _writer = new StreamWriter(_path, append: true);
            _writer.WriteLine($"=== {System.DateTime.Now:yyyy-MM-dd HH:mm:ss} Play started ===");
            _writer.Flush();
            Application.logMessageReceived += LogToFile;
            Debug.Log($"[UnityLogToFile] Writing log to: {_path}");
        }
        catch (System.Exception e)
        {
            Debug.LogError("[UnityLogToFile] Failed to open log file: " + e.Message);
        }
    }

    void OnDisable()
    {
        if (_writer == null) return;
        Application.logMessageReceived -= LogToFile;
        try
        {
            _writer.WriteLine($"=== {System.DateTime.Now:yyyy-MM-dd HH:mm:ss} Play stopped ===");
            _writer.Close();
        }
        catch { }
        _writer = null;
    }

    void LogToFile(string message, string stackTrace, LogType type)
    {
        if (_writer == null) return;
        try
        {
            _writer.WriteLine($"[{type}] {message}");
            if (!string.IsNullOrEmpty(stackTrace)) _writer.WriteLine(stackTrace);
            _writer.Flush();
        }
        catch { }
    }
}
