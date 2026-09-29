using System.Diagnostics;

namespace LsSpringTester;

/// <summary>
/// Bench-free simulator: slack, then a spring with rate ±4% of nominal and the drawing's
/// implied initial tension. Lets you exercise the whole UI/sequence without the frame.
/// </summary>
public sealed class SimulatedLsMachine : ILsMachine
{
    private readonly object _g = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly Random _rng = new();

    private double _lastT;
    private double _pos;            // true position, in (+ = extension)
    private double _vel;            // in/s
    private double? _target;        // true coords
    private double _posZero, _forceZero;

    private double _rateLbPerIn = 7.77, _initTensionLbf, _weightN = 0.03;
    private double _engageAt = 0.20;  // slack before the lower hook picks up the spring
    private bool _connected;

    public bool IsConnected => _connected;
    /// <summary>Test hook: makes the sim move faster than commanded (e.g. 60 = unit mismatch).</summary>
    public double SpeedMultiplier { get; set; } = 1.0;
    /// <summary>Emulates the LS1+ stopping a drive stage by itself (seconds; 0 = never).</summary>
    public double DriveDropoutSeconds { get; set; } = 1.1;
    /// <summary>Emulates the LS1+ taking a moment to start moving after a drive command.</summary>
    public double DriveStartLatencySeconds { get; set; } = 0.4;
    private double _driveStartT = -1;
    public event Action<string>? CommandSent;
    /// <summary>In the simulator, keep-alive simply suppresses the emulated drive dropouts.</summary>
    public bool KeepAliveDrives { get; set; }
    private double? _loadTargetN;    // load-mode drive target (tared N)

    public void LoadSpring(SpringSpec s)
    {
        lock (_g)
        {
            Update();
            _rateLbPerIn = s.RateLbPerIn * (1 + (_rng.NextDouble() * 2 - 1) * 0.04);
            double f0 = s.LoadNomLbf - s.RateLbPerIn * (s.TestLengthIn - s.NominalFreeLengthIn);
            _initTensionLbf = Math.Max(0, f0);
            _weightN = 0.01 + 0.002 * s.RateLbPerIn;
            _engageAt = _pos + 0.20;
        }
    }

    public Task ConnectAsync(CancellationToken ct) { _connected = true; return Task.CompletedTask; }
    public Task DisconnectAsync() { _connected = false; return Task.CompletedTask; }

    public async Task<MachineReading> ReadAsync(CancellationToken ct)
    {
        await Task.Delay(3, ct).ConfigureAwait(false);   // pretend serial latency
        lock (_g)
        {
            Update();
            double noise = (_rng.NextDouble() - 0.5) * 0.004;
            return new MachineReading(TrueForceN() - _forceZero + noise, _pos - _posZero, _vel != 0, DateTime.UtcNow);
        }
    }

    public Task StopAsync()
    {
        lock (_g) { Update(); _vel = 0; _target = null; _loadTargetN = null; }
        CommandSent?.Invoke("SIM stop");
        return Task.CompletedTask;
    }

    public const double SimSlowJogMmMin = 3, SimFastJogMmMin = 1500;   // measured on the LS1+

    public Task JogAsync(JogDirection dir, bool fast, CancellationToken ct)
    {
        lock (_g)
        {
            Update();
            _target = null;
            _loadTargetN = null;
            _vel = (dir == JogDirection.Extend ? 1 : -1) * (fast ? SimFastJogMmMin : SimSlowJogMmMin) / 25.4 / 60.0;
            _driveStartT = -1;
        }
        CommandSent?.Invoke($"SIM jog {dir} {(fast ? "fast" : "slow")}");
        return Task.CompletedTask;
    }

    public Task DriveToLoadAsync(double loadN, double speedInPerMin, CancellationToken ct)
    {
        lock (_g)
        {
            Update();
            _target = null;
            _loadTargetN = loadN;
            _vel = speedInPerMin / 60.0 * SpeedMultiplier;
            _driveStartT = _lastT;
        }
        CommandSent?.Invoke($"SIM drive-to-load {loadN:0.###} N @ {speedInPerMin * 25.4:0.#} mm/min");
        return Task.CompletedTask;
    }

    public Task MoveToAsync(double positionIn, double speedInPerMin, CancellationToken ct)
    {
        CommandSent?.Invoke($"SIM move-to {positionIn:0.0000} in @ {speedInPerMin * 25.4:0.#} mm/min");
        lock (_g)
        {
            Update();
            double tgt = positionIn + _posZero;
            if (Math.Abs(tgt - _pos) < 0.02) { _vel = 0; _target = null; return Task.CompletedTask; }   // LS1+ ignores tiny drives
            _target = tgt;
            _loadTargetN = null;
            _vel = Math.Sign(tgt - _pos) * speedInPerMin / 60.0 * SpeedMultiplier;
            _driveStartT = _lastT;
            if (_vel == 0) _target = null;
        }
        return Task.CompletedTask;
    }

    public double HomePositionIn { get { lock (_g) return -_posZero; } }

    public Task ZeroForceAsync(CancellationToken ct) { lock (_g) { Update(); _forceZero = TrueForceN(); } return Task.CompletedTask; }
    public Task ShiftForceZeroAsync(double deltaN) { lock (_g) _forceZero -= deltaN; return Task.CompletedTask; }
    public Task ZeroPositionAsync(CancellationToken ct) { lock (_g) { Update(); _posZero = _pos; } return Task.CompletedTask; }

    private double TrueForceN()
    {
        double x = _pos - _engageAt;
        double springLbf = x > 0 ? _initTensionLbf + _rateLbPerIn * x : 0;
        return _weightN + springLbf * SpringSpec.N_PER_LBF;
    }

    private void Update()
    {
        double t = _clock.Elapsed.TotalSeconds;
        double dt = t - _lastT;
        _lastT = t;
        if (_vel == 0) return;
        if (_driveStartT >= 0 && !KeepAliveDrives && DriveDropoutSeconds > 0 && t - _driveStartT > DriveDropoutSeconds + DriveStartLatencySeconds)
        {
            _vel = 0; _target = null; _loadTargetN = null; _driveStartT = -1;
            return;
        }
        if (_driveStartT >= 0 && t - _driveStartT < DriveStartLatencySeconds) return;   // not moving yet
        double step = _vel * dt;
        if (_target is double tgt)
        {
            double rem = tgt - _pos;
            if (Math.Abs(step) >= Math.Abs(rem)) { _pos = tgt; _vel = 0; _target = null; }
            else _pos += step;
        }
        else _pos += step;
        if (_loadTargetN is double lt && TrueForceN() - _forceZero >= lt) { _vel = 0; _loadTargetN = null; }
    }

    public void Dispose() { _connected = false; }
}
