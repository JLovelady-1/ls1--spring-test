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
        lock (_g) { Update(); _vel = 0; _target = null; }
        return Task.CompletedTask;
    }

    public Task JogAsync(JogDirection dir, double speedInPerMin, CancellationToken ct)
    {
        lock (_g)
        {
            Update();
            _target = null;
            _vel = (dir == JogDirection.Extend ? 1 : -1) * speedInPerMin / 60.0;
        }
        return Task.CompletedTask;
    }

    public Task MoveToAsync(double positionIn, double speedInPerMin, CancellationToken ct)
    {
        lock (_g)
        {
            Update();
            double tgt = positionIn + _posZero;
            _target = tgt;
            _vel = Math.Sign(tgt - _pos) * speedInPerMin / 60.0;
            if (_vel == 0) _target = null;
        }
        return Task.CompletedTask;
    }

    public Task HomeAsync(CancellationToken ct)
    {
        lock (_g)
        {
            Update();
            _target = 0;
            _vel = Math.Sign(0 - _pos) * 10.0 / 60.0;
            if (_vel == 0) _target = null;
        }
        return Task.CompletedTask;
    }

    public Task ZeroForceAsync(CancellationToken ct) { lock (_g) { Update(); _forceZero = TrueForceN(); } return Task.CompletedTask; }
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
        double step = _vel * dt;
        if (_target is double tgt)
        {
            double rem = tgt - _pos;
            if (Math.Abs(step) >= Math.Abs(rem)) { _pos = tgt; _vel = 0; _target = null; }
            else _pos += step;
        }
        else _pos += step;
    }

    public void Dispose() { _connected = false; }
}
