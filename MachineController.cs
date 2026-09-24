using System.Diagnostics;

namespace LsSpringTester;

public sealed class TestSettings
{
    public double PreloadN { get; set; } = 0.10;
    public double ApproachSpeedInPerMin { get; set; } = 0.5;
    public double TestSpeedInPerMin { get; set; } = 2.0;
    public double ReturnSpeedInPerMin { get; set; } = 4.0;
    public double JogSpeedInPerMin { get; set; } = 2.0;
    public double SettleSeconds { get; set; } = 2.0;
    public double SampleSeconds { get; set; } = 0.5;
    public double MaxPreloadSearchIn { get; set; } = 1.0;
    public double ForceLimitN { get; set; } = 100.0;      // set to your load cell capacity
    public double PositionToleranceIn { get; set; } = 0.002;
    public bool ZeroForceAtPreload { get; set; } = false;  // leave OFF (see notes)
}

public sealed record TestResult(
    SpringSpec Spec, double FreeLengthIn, double TravelIn,
    double MeasuredN, double MeasuredLbf, double FinalPositionIn,
    bool Pass, DateTime Time, int Samples);

/// <summary>
/// Owns all motion. Priority model:
///   1. StopAsync        – always runs immediately, cancels whatever is active, never queued.
///   2. ReturnToZero     – preempts: stop + cancel, then takes the operation lock.
///   3. Everything else  – rejected if another operation is active (no queuing, no overlap).
/// A background poll loop keeps <see cref="Latest"/> fresh; operations read that instead of
/// talking to the port themselves, so nothing competes with Stop for the machine.
/// </summary>
public sealed class MachineController : IDisposable
{
    public const int PollIntervalMs = 25;

    private readonly ILsMachine _machine;
    private readonly TestSettings _s;
    private readonly SemaphoreSlim _opLock = new(1, 1);
    private readonly object _ctsGate = new();
    private readonly object _readGate = new();

    private CancellationTokenSource? _opCts;
    private CancellationTokenSource? _pollCts;
    private Task? _pollTask;
    private MachineReading _latest;
    private volatile bool _jogRequested;
    private volatile bool _limitTripped;
    private volatile string? _activeOp;

    public MachineController(ILsMachine machine, TestSettings settings)
    {
        _machine = machine;
        _s = settings;
    }

    public event Action<MachineReading>? ReadingUpdated;
    public event Action<string>? StatusChanged;
    public event Action<bool>? BusyChanged;

    public MachineReading Latest { get { lock (_readGate) return _latest; } }
    public bool IsBusy => _activeOp != null;
    public string? ActiveOperation => _activeOp;

    // ───────────────────────── connection ─────────────────────────

    public async Task ConnectAsync()
    {
        await _machine.ConnectAsync(CancellationToken.None).ConfigureAwait(false);
        _pollCts = new CancellationTokenSource();
        var token = _pollCts.Token;
        _pollTask = Task.Run(() => PollLoopAsync(token));
        Status("Connected.");
    }

    public async Task DisconnectAsync()
    {
        await StopAsync("Disconnecting").ConfigureAwait(false);
        _pollCts?.Cancel();
        if (_pollTask != null) { try { await _pollTask.ConfigureAwait(false); } catch { } }
        await _machine.DisconnectAsync().ConfigureAwait(false);
        Status("Disconnected.");
    }

    // ───────────────────────── priority 1: STOP ─────────────────────────

    /// <summary>The single authoritative cancel path (CancelActiveMotion).</summary>
    public async Task StopAsync(string reason = "STOP")
    {
        _jogRequested = false;
        CancelActiveOperation();
        try { await _machine.StopAsync().ConfigureAwait(false); }
        catch (Exception ex) { Status($"Stop command error: {ex.Message}"); return; }
        Status($"{reason} — motion stopped.");
    }

    private void CancelActiveOperation()
    {
        lock (_ctsGate)
        {
            try { _opCts?.Cancel(); } catch (ObjectDisposedException) { }
        }
    }

    // ───────────────────────── priority 2: RETURN TO ZERO ─────────────────────────

    public Task<bool> ReturnToZeroAsync() => RunOpAsync("Return to zero", async ct =>
    {
        double dist = Math.Abs(Latest.PositionIn);
        Status($"Returning to zero at {_s.ReturnSpeedInPerMin:0.##} in/min…");
        await _machine.MoveToAsync(0.0, _s.ReturnSpeedInPerMin, ct).ConfigureAwait(false);
        var timeout = TimeSpan.FromSeconds(dist / _s.ReturnSpeedInPerMin * 60.0 * 1.5 + 10);
        await WaitForArrivalAsync(0.0, double.MaxValue, timeout, ct).ConfigureAwait(false);
        Status("At zero.");
        return true;
    }, preempt: true);

    // ───────────────────────── normal operations ─────────────────────────

    public Task<bool> TareForceAsync() => RunOpAsync("Tare force", async ct =>
    {
        await _machine.ZeroForceAsync(ct).ConfigureAwait(false);
        Status("Force tared with spring hanging free (spring weight removed).");
        return true;
    });

    public Task<bool> ZeroPositionAsync() => RunOpAsync("Zero position", async ct =>
    {
        await _machine.ZeroPositionAsync(ct).ConfigureAwait(false);
        Status("Position zeroed.");
        return true;
    });

    public Task<bool> HomeAsync() => RunOpAsync("Home", async ct =>
    {
        Status("Homing…");
        await _machine.HomeAsync(ct).ConfigureAwait(false);
        await WaitUntilStoppedAsync(TimeSpan.FromMinutes(3), ct).ConfigureAwait(false);
        Status("Home complete.");
        return true;
    });

    public Task<bool> JogAsync(JogDirection dir)
    {
        if (IsBusy) { Status($"Busy with {_activeOp} — jog ignored."); return Task.FromResult(false); }
        _jogRequested = true;
        return RunOpAsync("Jog", async ct =>
        {
            if (!_jogRequested) return false;             // released before we started
            await _machine.JogAsync(dir, _s.JogSpeedInPerMin, ct).ConfigureAwait(false);
            try
            {
                while (_jogRequested)
                {
                    ct.ThrowIfCancellationRequested();
                    if (dir == JogDirection.Extend && Latest.ForceN > _s.ForceLimitN)
                    {
                        Status("Jog stopped: force limit.");
                        break;
                    }
                    await Task.Delay(20, ct).ConfigureAwait(false);
                }
            }
            finally
            {
                await _machine.StopAsync().ConfigureAwait(false);
            }
            return true;
        });
    }

    public async Task JogReleaseAsync()
    {
        if (!_jogRequested) return;
        _jogRequested = false;
        if (_activeOp == "Jog")
        {
            try { await _machine.StopAsync().ConfigureAwait(false); } catch { }
        }
    }

    /// <summary>
    /// Creep in the extend direction until force ≥ preload, stop, then zero POSITION.
    /// Force is NOT re-zeroed by default, so the preload stays in the final reading.
    /// </summary>
    public Task<bool> FindPreloadAsync() => RunOpAsync("Find preload", async ct =>
    {
        double preload = _s.PreloadN;
        var start = Latest;
        if (start.ForceN >= preload)
        {
            Status($"Force already ≥ {preload:0.###} N — retract, re-tare, then find preload.");
            return false;
        }

        Status($"Approaching preload ({preload:0.###} N) at {_s.ApproachSpeedInPerMin:0.##} in/min…");
        await _machine.JogAsync(JogDirection.Extend, _s.ApproachSpeedInPerMin, ct).ConfigureAwait(false);

        bool found = false;
        try
        {
            while (true)
            {
                ct.ThrowIfCancellationRequested();
                var r = Latest;
                if (r.ForceN >= preload) { found = true; break; }
                if (r.PositionIn - start.PositionIn > _s.MaxPreloadSearchIn) break;
                await Task.Delay(5, ct).ConfigureAwait(false);
            }
        }
        finally
        {
            await _machine.StopAsync().ConfigureAwait(false);
        }

        if (!found)
        {
            Status($"No preload within {_s.MaxPreloadSearchIn:0.##} in of travel — is the lower hook attached?");
            return false;
        }

        await Task.Delay(300, ct).ConfigureAwait(false);          // let the crosshead come to rest
        double atPreload = Latest.ForceN;
        await _machine.ZeroPositionAsync(ct).ConfigureAwait(false);
        if (_s.ZeroForceAtPreload) await _machine.ZeroForceAsync(ct).ConfigureAwait(false);

        Status($"Preload reached ({atPreload:0.###} N). Position zeroed" +
               (_s.ZeroForceAtPreload ? " and force zeroed." : ".") + " Ready to run test.");
        return true;
    });

    /// <summary>
    /// One machine-side move to (test length − free length), stop, settle, average, compare.
    /// </summary>
    public Task<TestResult?> RunTestAsync(SpringSpec spec, double freeLengthIn) => RunOpAsync<TestResult?>("Run test", async ct =>
    {
        double travel = spec.TestLengthIn - freeLengthIn;
        if (travel <= 0 || travel > 3.0)
            throw new InvalidOperationException($"Travel {travel:0.000} in is out of range — check free length.");

        // Guard: never let a wrong-spring selection overload the cell.
        double guardN = Math.Min(_s.ForceLimitN, spec.LoadMaxLbf * SpringSpec.N_PER_LBF * 2.0 + 2.0);

        Status($"Driving {travel:0.000} in (to {spec.TestLengthIn:0.00} in length) at {_s.TestSpeedInPerMin:0.##} in/min…");
        await _machine.MoveToAsync(travel, _s.TestSpeedInPerMin, ct).ConfigureAwait(false);

        double expectedSec = travel / _s.TestSpeedInPerMin * 60.0;
        await WaitForArrivalAsync(travel, guardN, TimeSpan.FromSeconds(expectedSec * 1.5 + 10), ct).ConfigureAwait(false);

        Status($"At position. Settling {_s.SettleSeconds:0.#} s…");
        await Task.Delay(TimeSpan.FromSeconds(_s.SettleSeconds), ct).ConfigureAwait(false);

        Status("Sampling…");
        var (avgN, n) = await SampleForceAsync(TimeSpan.FromSeconds(_s.SampleSeconds), ct).ConfigureAwait(false);

        double lbf = avgN * SpringSpec.N_TO_LBS;
        bool pass = lbf >= spec.LoadMinLbf && lbf <= spec.LoadMaxLbf;
        var result = new TestResult(spec, freeLengthIn, travel, avgN, lbf, Latest.PositionIn, pass, DateTime.Now, n);

        Status($"{spec.PartNumber}: {lbf:0.000} lbf ({avgN:0.000} N) — {(pass ? "PASS" : "FAIL")}  " +
               $"[spec {spec.LoadMinLbf:0.000} – {spec.LoadMaxLbf:0.000} lbf]");
        return result;
    });

    // ───────────────────────── plumbing ─────────────────────────

    private async Task<T> RunOpAsync<T>(string name, Func<CancellationToken, Task<T>> body, bool preempt = false)
    {
        if (preempt)
        {
            await StopAsync($"{name} requested").ConfigureAwait(false);
            if (!await _opLock.WaitAsync(TimeSpan.FromSeconds(5)).ConfigureAwait(false))
            {
                Status($"{name}: previous operation did not release — try again.");
                return default!;
            }
        }
        else if (!await _opLock.WaitAsync(0).ConfigureAwait(false))
        {
            Status($"Busy with {_activeOp} — {name} ignored.");
            return default!;
        }

        var cts = new CancellationTokenSource();
        lock (_ctsGate) _opCts = cts;
        _activeOp = name;
        BusyChanged?.Invoke(true);
        try
        {
            return await body(cts.Token).ConfigureAwait(false);
        }
        catch (OperationCanceledException)
        {
            Status($"{name} cancelled.");
            return default!;
        }
        catch (Exception ex)
        {
            try { await _machine.StopAsync().ConfigureAwait(false); } catch { }
            Status($"{name} aborted: {ex.Message}");
            return default!;
        }
        finally
        {
            lock (_ctsGate) { if (ReferenceEquals(_opCts, cts)) _opCts = null; }
            _activeOp = null;
            _opLock.Release();
            BusyChanged?.Invoke(false);
        }
    }

    private async Task WaitForArrivalAsync(double target, double guardN, TimeSpan timeout, CancellationToken ct)
    {
        var sw = Stopwatch.StartNew();
        bool sawMoving = false;
        while (true)
        {
            ct.ThrowIfCancellationRequested();
            var r = Latest;

            if (Math.Abs(r.ForceN) > guardN)
            {
                await _machine.StopAsync().ConfigureAwait(false);
                throw new InvalidOperationException(
                    $"Force {r.ForceN:0.00} N exceeded guard {guardN:0.00} N — wrong spring selected?");
            }

            if (r.IsMoving) sawMoving = true;
            bool atTarget = Math.Abs(r.PositionIn - target) <= _s.PositionToleranceIn;
            long ms = sw.ElapsedMilliseconds;

            if (!r.IsMoving && atTarget && (sawMoving || ms > 500)) return;

            if (!r.IsMoving && !atTarget && ms > 300 && (sawMoving || ms > 1500))
                throw new InvalidOperationException(
                    $"Crosshead stopped at {r.PositionIn:0.0000} in, short of {target:0.0000} in.");

            if (sw.Elapsed > timeout)
            {
                await _machine.StopAsync().ConfigureAwait(false);
                throw new TimeoutException($"Move to {target:0.000} in timed out.");
            }

            await Task.Delay(10, ct).ConfigureAwait(false);
        }
    }

    private async Task WaitUntilStoppedAsync(TimeSpan timeout, CancellationToken ct)
    {
        var sw = Stopwatch.StartNew();
        await Task.Delay(300, ct).ConfigureAwait(false);
        while (Latest.IsMoving)
        {
            if (sw.Elapsed > timeout)
            {
                await _machine.StopAsync().ConfigureAwait(false);
                throw new TimeoutException("Motion timed out.");
            }
            await Task.Delay(20, ct).ConfigureAwait(false);
        }
    }

    private async Task<(double avgN, int n)> SampleForceAsync(TimeSpan window, CancellationToken ct)
    {
        var sw = Stopwatch.StartNew();
        double sum = 0; int n = 0; DateTime last = default;
        while (sw.Elapsed < window || n == 0)
        {
            ct.ThrowIfCancellationRequested();
            if (n == 0 && sw.Elapsed > window + TimeSpan.FromSeconds(2))
                throw new InvalidOperationException("No force readings received while sampling.");
            var r = Latest;
            if (r.Time != last) { sum += r.ForceN; n++; last = r.Time; }
            await Task.Delay(5, ct).ConfigureAwait(false);
        }
        return (sum / n, n);
    }

    private async Task PollLoopAsync(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            try
            {
                var r = await _machine.ReadAsync(ct).ConfigureAwait(false);
                lock (_readGate) _latest = r;
                ReadingUpdated?.Invoke(r);

                // Global over-force watchdog (fires once per exceedance so Return can still run).
                if (r.ForceN > _s.ForceLimitN)
                {
                    if (!_limitTripped)
                    {
                        _limitTripped = true;
                        _ = StopAsync($"FORCE LIMIT {_s.ForceLimitN:0.#} N");
                    }
                }
                else if (r.ForceN < _s.ForceLimitN * 0.9)
                {
                    _limitTripped = false;
                }
            }
            catch (OperationCanceledException) when (ct.IsCancellationRequested) { break; }
            catch (Exception ex)
            {
                Status($"Read error: {ex.Message}");
                await SafeDelay(500, ct).ConfigureAwait(false);
            }
            await SafeDelay(PollIntervalMs, ct).ConfigureAwait(false);
        }
    }

    private static async Task SafeDelay(int ms, CancellationToken ct)
    {
        try { await Task.Delay(ms, ct).ConfigureAwait(false); } catch (OperationCanceledException) { }
    }

    private void Status(string msg) => StatusChanged?.Invoke(msg);

    public void Dispose()
    {
        _jogRequested = false;
        CancelActiveOperation();
        _pollCts?.Cancel();
        try { _machine.StopAsync().Wait(500); } catch { }
        _machine.Dispose();
    }
}
