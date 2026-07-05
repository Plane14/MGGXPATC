using System.Text.Json;

namespace Atc.Daemon;

public class AtcdConfiguration
{
    public string[] FirIcaoCodes { get; set; } = Array.Empty<string>();
    public ServiceEndpointConfig ServiceEndpoint { get; set; } = new();
    public TelemetryEndpointConfig TelemetryEndpoint { get; set; } = new();

    public static AtcdConfiguration Load()
    {
        const string configFileName = "appsettings.json";
        if (!File.Exists(configFileName))
        {
            Console.WriteLine($"WARNING: {configFileName} not found, using default configuration.");
            return new AtcdConfiguration();
        }

        var json = File.ReadAllText(configFileName);
        var config = JsonSerializer.Deserialize<AtcdConfiguration>(json);
        return config ?? new AtcdConfiguration();
    }
}

public class ServiceEndpointConfig
{
    public int Port { get; set; } = 3001;
    public string Path { get; set; } = "/atc";
}

public class TelemetryEndpointConfig
{
    public int Port { get; set; } = 3003;
    public string Path { get; set; } = "/telemetry";
    public string LogLevel { get; set; } = "Debug";
    public int DelayBeforeFirstPushSeconds { get; set; } = 3;
}
