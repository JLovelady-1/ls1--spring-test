using System.Diagnostics;

namespace LsSpringTester;

public sealed class TestSettings
{
    public double PreloadN { get; set; } = 0.10;
    public const double MM_PER_IN = 25.4;
    public double ApproachSpeedInPerMin { get; set; } = 100.0 / MM_PER_IN;  // preload speed, 100 mm/min
    public double TestSpeedInPerMin { get; set; } = 100.0 / MM_PER_IN;      // test speed, 100 mm/min
    public double ReturnSpeedInPerMin { get; set; } = 20.0 / MM_PER_IN;     // jog-mode return
    public double ReturnFastMmMin { get; set; } = 1200;   // AQM app: 20 (mm/s) → 1200 mm/min until close
    public double ReturnSlowMmMin { get; set; } = 60;     // AQM app: ~1 mm/s for the last 5 mm
    public double ReturnSlowZoneMm { get; set; } = 5;
    public double ReturnTargetMm { get; set; } = 0;
    /// <summary>Two-point rate: points at these % of the full travel (preload → test length).</summary>
    public double RatePoint1Pct { get; set; } = 20;
    public double RatePoint2Pct { get; set; } = 60;       // RETURN goes to this crosshead distance (mm, from zero)
    public double PreloadDwellSeconds { get; set; } = 1.0;                  // stop + settle at preload before zeroing
    public double JogSpeedInPerMin { get; set; } = 2.0;
    public double SettleSeconds { get; set; } = 1.0;
    public double SampleSeconds { get; set; } = 0.5;
    public double MaxPreloadSearchIn { get; set; } = 20.0 / MM_PER_IN;   // max travel hunting for 0 N / preload (0 = no limit)
    public bool AutoPreload { get; set; } = false;            // preload above the spring's initial tension (per spring)
    public bool PreloadFromZero { get; set; } = true;        // RUN TEST: drive down to 0 N first, then up to preload
    public double PreloadFractionOfRange { get; set; } = 0.10;
    public double ForceLimitN { get; set; } = 100.0;      // set to your load cell capacity
    public double PositionToleranceIn { get; set; } = 0.002;   // ≈0.05 mm (AQM HOME_TOL_MM)
    public double StaleReadingSeconds { get; set; } = 1.0;
    public double PreloadOvershootN { get; set; } = 2.0;      // abort if preload overshoots by more than this

    /// <summary>
    /// false = MtmRunDriveStage at the approach/test speeds (rate sent in mm/s — see LsSerialMachine).
    /// true  = fallback: machine SLOW JOG (CtlJogMachine), stopping at the target.
    /// </summary>
    public bool UseJogMotion { get; set; } = false;
    public bool PreloadUseJog { get; set; } = false;           // true = slow jog (~3 mm/min, continuous); false = drive at ApproachSpeed
    public double FinishZoneIn { get; set; } = 0.03;           // frame ignores drives this close → finish with slow jog
    public double MaxAutoSpeedMmMin { get; set; } = 150;      // jog-mode safety: stop if slow jog exceeds this
    public double FastApproachZoneIn { get; set; } = 0.12;    // fast jog until ~3 mm from target, then slow jog
    public bool TestFastApproach { get; set; } = false;       // RUN TEST: fast jog for the bulk (LS1+ fast jog ≈ 1500 mm/min — too fast)
    public bool ZeroForceAtPreload { get; set; } = true;   // zero force AND position at preload; load result adds the preload back
    public bool MeasureRate { get; set; } = true;          // two-point rate between the drawing's two load lengths
}

/// <summary>One settled step: position from preload zero (in) and force (lbf, zeroed at preload).</summary>
public readonly record struct RatePoint(double PositionIn, double ForceLbf);

public sealed record TestResult(
    SpringSpec Spec, double FreeLengthIn, double TravelIn,
    double MeasuredN, double MeasuredLbf, double FinalPositionIn,
    bool LoadPass, double? RateLbPerIn, bool? RatePass,
    DateTime Time, int Samples, IReadOnlyList<RatePoint>? Points = null)
{
    /// <summary>Pass/fail is based on the measured spring rate only.</summary>
    public bool Pass => RatePass == true;
}

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
    public double ReadingsHz { get; private set; }

    /// <summary>Force removed by zeroing at the preload; added back to the absolute load reading.</summary>
    private double _preloadOffsetN;

    /// <summary>
    /// The ZERO button's position, stored relative to machine home so it survives the re-zero at preload.
    /// null until ZERO is pressed (then the current zero is used).
    /// </summary>
    private double? _startRelHomeIn;

    private double StartPositionIn => _startRelHomeIn is double rel ? _machine.HomePositionIn + rel : 0.0;

    // ───────────────────────── connection ─────────────────────────

    public async Task ConnectAsync()
    {
        _machine.CommandSent += cmd => Status(cmd);
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
        double targetMm = _s.ReturnTargetMm;
        Status($"Returning to {targetMm:0.##} mm at {_s.ReturnFastMmMin:0} mm/min…");
        await ReturnMoveAsync(targetMm / TestSettings.MM_PER_IN, ct).ConfigureAwait(false);
        Status($"At {targetMm:0.##} mm.");
        return true;
    }, preempt: true);

    // ───────────────────────── normal operations ─────────────────────────

    public Task<bool> TareForceAsync() => RunOpAsync("Tare force", async ct =>
    {
        await _machine.ZeroForceAsync(ct).ConfigureAwait(false);
        await NextReadingAsync(ct).ConfigureAwait(false);
        Status("Force tared with spring hanging free (spring weight removed).");
        return true;
    });

    /// <summary>ZERO button: tare force (spring hanging free) and zero position.</summary>
    public Task<bool> ZeroAllAsync() => RunOpAsync("Zero", async ct =>
    {
        await _machine.ZeroForceAsync(ct).ConfigureAwait(false);
        await _machine.ZeroPositionAsync(ct).ConfigureAwait(false);
        await NextReadingAsync(ct).ConfigureAwait(false);
        _startRelHomeIn = 0.0 - _machine.HomePositionIn;          // remember this spot as the slack start point
        Status("Force and position zeroed — this is the start position for RUN TEST.");
        return true;
    });

    public Task<bool> ZeroPositionAsync() => RunOpAsync("Zero position", async ct =>
    {
        await _machine.ZeroPositionAsync(ct).ConfigureAwait(false);
        await NextReadingAsync(ct).ConfigureAwait(false);
        Status("Position zeroed.");
        return true;
    });

    public Task<bool> HomeAsync() => RunOpAsync("Home", async ct =>
    {
        Status("Moving to machine home (machine position 0)…");
        await ReturnMoveAsync(_machine.HomePositionIn, ct).ConfigureAwait(false);
        Status("Home complete.");
        return true;
    });

    public Task<bool> JogAsync(JogDirection dir, bool fast = false)
    {
        if (IsBusy) { Status($"Busy with {_activeOp} — jog ignored."); return Task.FromResult(false); }
        _jogRequested = true;
        return RunOpAsync("Jog", async ct =>
        {
            if (!_jogRequested) return false;             // released before we started
            await _machine.JogAsync(dir, fast, ct).ConfigureAwait(false);
            double jogStart = Latest.PositionIn;
            var jogSw = Stopwatch.StartNew();
            bool warned = false;
            try
            {
                while (_jogRequested)
                {
                    if (!warned && jogSw.ElapsedMilliseconds > 2000 && Math.Abs(Latest.PositionIn - jogStart) < 0.0005)
                    {
                        warned = true;
                        Status("Jog sent but the crosshead isn't moving — check E-stop / limits, or power-cycle the LS1+.");
                    }
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
    /// Drive UP if force is below the preload, DOWN if above, stop when it crosses, settle,
    /// then zero POSITION. Force is not re-zeroed, so the preload stays in the final reading.
    /// </summary>
    /// <summary>One button: find preload (settle, zero force + position), then the 3-step rate test.</summary>
    public Task<TestResult?> RunSequenceAsync(SpringSpec spec, double freeLengthIn) =>
        RunOpAsync<TestResult?>("Run test", async ct =>
        {
            await RestoreForceZeroAsync(ct).ConfigureAwait(false);   // in case a previous run was aborted mid-test
            if (_s.PreloadFromZero)
            {
                // Unload first: drive DOWN until the force reads ~0 (spring slack), whatever load it was hooked in with.
                const double zeroBandN = 0.02;
                if (FreshLatest().ForceN > zeroBandN)
                {
                    Status("Unloading to 0 N before preload…");
                    if (!await ApproachForceAsync(0.0, up: false, zeroBandN, ct).ConfigureAwait(false)) return null;
                    await WaitStoppedAsync(ct).ConfigureAwait(false);
                }
            }
            if (!await FindPreloadCoreAsync(spec, ct).ConfigureAwait(false)) return null;
            return await RunTestCoreAsync(spec, freeLengthIn, ct).ConfigureAwait(false);
        });

    public Task<bool> FindPreloadAsync() => RunOpAsync("Find preload", FindPreloadCoreAsync);

    /// <summary>
    /// Preload that guarantees the spring is really engaged: above the drawing's implied initial tension
    /// (load at test length − rate × travel) plus 10 % of the working range, never below the Setup minimum.
    /// </summary>
    public double PreloadFor(SpringSpec? spec)
    {
        if (spec == null || !_s.AutoPreload) return _s.PreloadN;
        double span = spec.TestLengthIn - spec.NominalFreeLengthIn;
        double initialTensionLbf = Math.Max(0, spec.LoadNomLbf - spec.RateLbPerIn * span);
        double lbf = initialTensionLbf + _s.PreloadFractionOfRange * spec.RateLbPerIn * span;
        return Math.Max(_s.PreloadN, lbf * SpringSpec.N_PER_LBF);
    }

    private Task<bool> FindPreloadCoreAsync(CancellationToken ct) => FindPreloadCoreAsync(null, ct);

    /// <summary>
    /// If above the preload, drive DOWN past it; then always finish by driving UP to it (same direction as
    /// the test, so it never settles in slack). Settle, then zero force + position.
    /// </summary>
    private async Task<bool> FindPreloadCoreAsync(SpringSpec? spec, CancellationToken ct)
    {
        double preload = PreloadFor(spec);
        double band = Math.Max(0.01, preload * 0.05);
        Status($"Preload target {preload:0.###} N ({preload * SpringSpec.N_TO_LBS:0.000} lbf)" +
               (spec != null && _s.AutoPreload ? " — auto, above initial tension." : "."));

        if (FreshLatest().ForceN > preload + band)
        {
            if (!await ApproachForceAsync(preload, up: false, band, ct).ConfigureAwait(false)) return false;
            await WaitStoppedAsync(ct).ConfigureAwait(false);
        }
        if (FreshLatest().ForceN < preload - band)
        {
            if (!await ApproachForceAsync(preload, up: true, band, ct).ConfigureAwait(false)) return false;
            await Task.Delay(200, ct).ConfigureAwait(false);
            double peak = FreshLatest().ForceN;
            if (peak > preload + _s.PreloadOvershootN)
                throw new InvalidOperationException(
                    $"Overshot preload: {peak:0.00} N vs {preload:0.###} N target. Stopped — lower the preload speed.");
        }

        Status($"Holding {_s.PreloadDwellSeconds:0.#} s to settle…");
        await Task.Delay(TimeSpan.FromSeconds(_s.PreloadDwellSeconds), ct).ConfigureAwait(false);
        double atPreload = FreshLatest().ForceN;
        if (atPreload < -preload)
            throw new InvalidOperationException(
                $"Force reads NEGATIVE in tension ({atPreload:0.###} N). Set FORCE_SIGN = -1.0 in LsSerialMachine.cs.");
        if (atPreload < preload * 0.3)
            throw new InvalidOperationException(
                $"Force relaxed to {atPreload:0.###} N after settling — spring may not be engaged. Check the hooks and retry.");
        await _machine.ZeroPositionAsync(ct).ConfigureAwait(false);
        _preloadOffsetN = 0;
        if (_s.ZeroForceAtPreload)
        {
            await _machine.ZeroForceAsync(ct).ConfigureAwait(false);
            _preloadOffsetN = atPreload;
        }
        await NextReadingAsync(ct).ConfigureAwait(false);   // don't let a pre-zero sample look like motion

        Status($"Preload settled at {atPreload:0.###} N. Position zeroed" +
               (_s.ZeroForceAtPreload ? " and force zeroed." : ".") + " Spring engaged — measuring rate.");
        return true;
    }

    /// <summary>Drive in one direction until force crosses the target (±band). Returns false if not reached.</summary>
    private async Task<bool> ApproachForceAsync(double target, bool up, double band, CancellationToken ct)
    {
        var start = FreshLatest();
        bool useJog = _s.UseJogMotion || _s.PreloadUseJog;
        var dir = up ? JogDirection.Extend : JogDirection.Retract;
        Status($"Force {start.ForceN:0.###} N → driving {(up ? "UP" : "DOWN")} to {target:0.###} N" +
               (useJog ? " (slow jog)…" : $" at {_s.ApproachSpeedInPerMin * TestSettings.MM_PER_IN:0.#} mm/min…"));

        double limitIn = _s.MaxPreloadSearchIn > 0 ? _s.MaxPreloadSearchIn : 5.0;   // "no limit" → 127 mm drive target
        double searchTarget = start.PositionIn + (up ? 1 : -1) * limitIn;
        async Task Issue()
        {
            if (useJog) await _machine.JogAsync(dir, false, ct).ConfigureAwait(false);
            else
            {
                await WaitStoppedAsync(ct).ConfigureAwait(false);
                await _machine.MoveToAsync(searchTarget, _s.ApproachSpeedInPerMin, ct).ConfigureAwait(false);
            }
        }
        await Issue().ConfigureAwait(false);

        var speedMon = new SpeedMonitor();
        var grace = Stopwatch.StartNew();
        var sinceProgress = Stopwatch.StartNew();
        double lastPos = start.PositionIn, posAtResume = start.PositionIn;
        int idleResumes = 0;
        bool movedSinceIssue = false;
        try
        {
            while (true)
            {
                ct.ThrowIfCancellationRequested();
                var r = FreshLatest();
                if (grace.ElapsedMilliseconds > 600) CheckSpeed(speedMon, r, useJog ? null : _s.ApproachSpeedInPerMin);
                else speedMon.MmPerMin(r);

                if (up ? r.ForceN >= target - band : r.ForceN <= target + band) return true;
                if (_s.MaxPreloadSearchIn > 0 && Math.Abs(r.PositionIn - start.PositionIn) >= _s.MaxPreloadSearchIn - 0.003)
                {
                    Status($"{(up ? "Preload" : "0 N")} not reached within {_s.MaxPreloadSearchIn * TestSettings.MM_PER_IN:0.#} mm " +
                           "(Setup → Max search distance; 0 = no limit).");
                    return false;
                }

                if (Math.Abs(r.PositionIn - lastPos) > 0.0001) { lastPos = r.PositionIn; sinceProgress.Restart(); movedSinceIssue = true; }
                else if (sinceProgress.ElapsedMilliseconds > ResumeAfterMs(movedSinceIssue))
                {
                    idleResumes = Math.Abs(r.PositionIn - posAtResume) < 0.0005 ? idleResumes + 1 : 0;
                    if (idleResumes >= 2) throw new InvalidOperationException(NotRespondingMsg);
                    posAtResume = r.PositionIn;
                    movedSinceIssue = false;
                    await Issue().ConfigureAwait(false);
                    sinceProgress.Restart();
                    grace.Restart();
                    speedMon = new SpeedMonitor();
                }
                await Task.Delay(5, ct).ConfigureAwait(false);
            }
        }
        finally
        {
            await _machine.StopAsync().ConfigureAwait(false);
        }
    }


    /// <summary>
    /// One machine-side move to (test length − free length), stop, settle, average, compare.
    /// </summary>
    /// <summary>
    /// After FIND PRELOAD (force + position zeroed), step the spring to 3 lengths evenly spaced between the
    /// drawing's two load lengths (2.37 / 2.52 / 2.67 in main, 2.35 / 2.55 / 2.75 in trim). At each step:
    /// stop, settle, average force + position. Rate = best-fit slope through the 3 points (initial tension
    /// in trim springs doesn't bias it). Load pass/fail uses step 3 (the test length) plus the preload.
    /// </summary>
    public Task<TestResult?> RunTestAsync(SpringSpec spec, double freeLengthIn) =>
        RunOpAsync<TestResult?>("Run test", ct => RunTestCoreAsync(spec, freeLengthIn, ct));

    private async Task<TestResult?> RunTestCoreAsync(SpringSpec spec, double freeLengthIn, CancellationToken ct)
    {
        double travel = spec.TestLengthIn - freeLengthIn;
        if (travel <= 0 || travel > 3.0)
            throw new InvalidOperationException($"Travel {travel:0.000} in is out of range — check free length.");

        double guardN = Math.Min(_s.ForceLimitN, spec.LoadMaxLbf * SpringSpec.N_PER_LBF * 2.0 + 2.0);

        // Two-point rate at 20 % and 60 % of the full travel from the preload point
        // (trim: 0.20 / 0.60 in of 1.00 in; main: 0.084 / 0.252 in of 0.42 in).
        var targets = new List<double> { travel * _s.RatePoint1Pct / 100.0, travel * _s.RatePoint2Pct / 100.0 };

        var points = new List<RatePoint>();
        double lastN = 0, lastPos = 0; int n = 0;
        for (int i = 0; i < targets.Count; i++)
        {
            string label = $"Point {i + 1}/{targets.Count} ({(i == 0 ? _s.RatePoint1Pct : _s.RatePoint2Pct):0}%)";
            Status($"{label}: moving to {targets[i]:0.000} in from preload{SpeedText()}");
            await MoveExactAsync(targets[i], _s.TestSpeedInPerMin, guardN, ct, allowFast: _s.TestFastApproach).ConfigureAwait(false);
            Status($"{label}: settling {_s.SettleSeconds:0.#} s…");
            await Task.Delay(TimeSpan.FromSeconds(_s.SettleSeconds), ct).ConfigureAwait(false);
            (lastN, lastPos, n) = await SampleAsync(TimeSpan.FromSeconds(_s.SampleSeconds), ct).ConfigureAwait(false);
            points.Add(new RatePoint(lastPos, lastN * SpringSpec.N_TO_LBS));
            Status($"{label}: {lastN * SpringSpec.N_TO_LBS:0.000} lbf at {lastPos:0.0000} in");
        }

        double lbf = (lastN + _preloadOffsetN) * SpringSpec.N_TO_LBS;       // absolute load incl. preload
        bool loadPass = lbf >= spec.LoadMinLbf && lbf <= spec.LoadMaxLbf;

        double? rate = null; bool? ratePass = null;
        if (points.Count >= 2 && points[^1].PositionIn - points[0].PositionIn > 0.02)
        {
            rate = BestFitSlope(points);
            ratePass = rate >= spec.RateMinLbPerIn && rate <= spec.RateMaxLbPerIn;
        }
        else Status("Rate not computed — the steps were too close together.");

        var result = new TestResult(spec, freeLengthIn, travel, lastN + _preloadOffsetN, lbf, lastPos,
                                    loadPass, rate, ratePass, DateTime.Now, n, points);
        await RestoreForceZeroAsync(ct).ConfigureAwait(false);   // back to the ZERO (spring-free) force reference

        Status($"{spec.PartNumber}: rate " + (rate is double rr ? $"{rr:0.000} lb/in" : "—") +
               $"  [{spec.RateMinLbPerIn:0.000}–{spec.RateMaxLbPerIn:0.000}]  → {(result.Pass ? "PASS" : "FAIL")}");
        return result;
    }


    /// <summary>Least-squares slope of force (lbf) vs position (in).</summary>
    private static double BestFitSlope(IReadOnlyList<RatePoint> pts)
    {
        double mx = pts.Average(p => p.PositionIn), my = pts.Average(p => p.ForceLbf);
        double sxy = pts.Sum(p => (p.PositionIn - mx) * (p.ForceLbf - my));
        double sxx = pts.Sum(p => (p.PositionIn - mx) * (p.PositionIn - mx));
        return sxy / sxx;
    }

    private string SpeedText() =>
        !_s.UseJogMotion ? $" at {_s.TestSpeedInPerMin * TestSettings.MM_PER_IN:0.#} mm/min…"
        : _s.TestFastApproach ? $" — fast jog, slow jog for the last {_s.FastApproachZoneIn * TestSettings.MM_PER_IN:0.#} mm…"
        : " at slow-jog speed…";

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

    private enum MoveOutcome { Arrived, Stalled, Overshot, Near }

    /// <summary>
    /// Re-issue a drive the LS1+ has dropped. It takes ~0.4 s to start moving after a command, so a drive
    /// that has not started yet gets 1.5 s; one that moved and then stopped is re-issued after 0.3 s.
    /// </summary>
    private static int ResumeAfterMs(bool movedSinceIssue) => movedSinceIssue ? 250 : 2000;

    private const string NotRespondingMsg =
        "Crosshead is not responding to drive commands. Stopped. Check E-stop / limits; if jog also doesn't move, " +
        "power-cycle the LS1+ and reconnect.";

    /// <summary>
    /// Jog mode: slow jog to the target (optionally fast jog first for long return/home moves),
    ///           stopping early by the measured coast distance; overshoots are corrected by jogging back.
    /// Drive mode: fast drive to within 0.02 in, then a slow final drive.
    /// Every motion start stops + settles first (no stacked commands).
    /// </summary>
    /// <summary>
    /// Like the AQM app's Return Home: fast drive (20 mm/s) until 5 mm away, then 60 mm/min to the target.
    /// </summary>
    /// <summary>
    /// Like the AQM app's Return Home: one drive straight to the target at the return speed (the frame stops
    /// at the target itself), then a gentle finish only if it ended outside tolerance.
    /// </summary>
    private async Task ReturnMoveAsync(double target, CancellationToken ct)
    {
        if (!_s.UseJogMotion)
        {
            var o = await DriveAndWaitAsync(target, _s.ReturnFastMmMin / TestSettings.MM_PER_IN,
                                            _s.PositionToleranceIn, double.MaxValue, ct, stopAtTarget: false).ConfigureAwait(false);
            await WaitStoppedAsync(ct).ConfigureAwait(false);
            if (o == MoveOutcome.Arrived && Math.Abs(FreshLatest().PositionIn - target) <= _s.PositionToleranceIn * 2) return;
            await MoveExactAsync(target, _s.ReturnSlowMmMin / TestSettings.MM_PER_IN, double.MaxValue, ct).ConfigureAwait(false);
        }
        else await MoveExactAsync(target, _s.ReturnSpeedInPerMin, double.MaxValue, ct, allowFast: true).ConfigureAwait(false);
    }

    private async Task MoveExactAsync(double target, double speedInPerMin, double guardN, CancellationToken ct, bool allowFast = false)
    {
        double tol = _s.PositionToleranceIn;

        if (!_s.UseJogMotion)
        {
            // Drive the whole way at the set speed; the LS1+ ignores drives < ~0.5 mm, so any small
            // remainder (short stop / overshoot) is finished with slow jog.
            var o = await DriveAndWaitAsync(target, speedInPerMin, tol, guardN, ct).ConfigureAwait(false);
            if (o == MoveOutcome.Arrived && await SettledWithinAsync(target, tol * 2, ct).ConfigureAwait(false)) return;
        }
        else
        {
            double dist = target - FreshLatest().PositionIn;
            double zone = _s.FastApproachZoneIn;
            if (allowFast && Math.Abs(dist) > zone * 1.5)
            {
                await DriveAndWaitAsync(target - Math.Sign(dist) * zone, 0, zone * 0.25, guardN, ct, fastJog: true).ConfigureAwait(false);
                await WaitStoppedAsync(ct).ConfigureAwait(false);
            }
        }

        for (int attempt = 0; attempt < 4; attempt++)
        {
            var outcome = await DriveAndWaitAsync(target, 0, tol, guardN, ct, forceJog: true).ConfigureAwait(false);
            if (outcome == MoveOutcome.Stalled) throw new InvalidOperationException(NotRespondingMsg);
            if (outcome == MoveOutcome.Arrived && await SettledWithinAsync(target, tol * 2, ct).ConfigureAwait(false)) return;
        }
        throw new InvalidOperationException(
            $"Could not settle within ±{tol:0.0000} in of {target:0.0000} in (at {Latest.PositionIn:0.0000} in).");
    }

    private async Task<bool> SettledWithinAsync(double target, double tol, CancellationToken ct)
    {
        await Task.Delay(300, ct).ConfigureAwait(false);
        return Math.Abs(FreshLatest().PositionIn - target) <= tol;
    }

    private async Task<MoveOutcome> DriveAndWaitAsync(double target, double speedInPerMin, double tol, double guardN,
                                                      CancellationToken ct, bool fastJog = false, bool forceJog = false,
                                                      bool stopAtTarget = true)
    {
        double startPos = FreshLatest().PositionIn;
        if (Math.Abs(target - startPos) <= tol) return MoveOutcome.Arrived;
        int dir = Math.Sign(target - startPos);
        bool jog = _s.UseJogMotion || forceJog || fastJog;

        if (jog) await _machine.JogAsync(dir > 0 ? JogDirection.Extend : JogDirection.Retract, fastJog, ct).ConfigureAwait(false);
        else
        {
            await WaitStoppedAsync(ct).ConfigureAwait(false);   // never stack a drive on a crosshead still decelerating
            await _machine.MoveToAsync(target, speedInPerMin, ct).ConfigureAwait(false);
        }

        var timeout = jog ? TimeSpan.FromMinutes(10)
                          : TimeSpan.FromSeconds(Math.Abs(target - startPos) / speedInPerMin * 60.0 * 1.5 + 10);
        var sw = Stopwatch.StartNew();
        var grace = Stopwatch.StartNew();
        var sinceProgress = Stopwatch.StartNew();
        double lastPos = startPos, posAtResume = startPos;
        int idleResumes = 0;
        bool movedSinceIssue = false;
        var speedMon = new SpeedMonitor();

        while (true)
        {
            ct.ThrowIfCancellationRequested();
            var r = FreshLatest();

            if (Math.Abs(r.ForceN) > guardN)
            {
                await _machine.StopAsync().ConfigureAwait(false);
                throw new InvalidOperationException(
                    $"Force {r.ForceN:0.00} N exceeded guard {guardN:0.00} N — wrong spring selected?");
            }

            // Skip the first 0.6 s: acceleration / coast from the previous motion isn't this move's speed.
            if (!fastJog && grace.ElapsedMilliseconds > 600) CheckSpeed(speedMon, r, jog ? null : speedInPerMin);
            else speedMon.MmPerMin(r);

            // In jog mode, stop early by the distance the crosshead coasts (~80 ms at current speed).
            double lead = jog ? (speedMon.Last ?? 0) / 60.0 / TestSettings.MM_PER_IN * 0.08 : 0;
            double err = target - r.PositionIn;
            if (Math.Abs(err) <= tol + lead)
            {
                if (stopAtTarget || jog) await _machine.StopAsync().ConfigureAwait(false);
                return MoveOutcome.Arrived;
            }
            if (Math.Sign(err) != dir) { await _machine.StopAsync().ConfigureAwait(false); return MoveOutcome.Overshot; }

            if (Math.Abs(r.PositionIn - lastPos) > 0.0001) { lastPos = r.PositionIn; sinceProgress.Restart(); movedSinceIssue = true; }
            else if (!jog && sinceProgress.ElapsedMilliseconds > ResumeAfterMs(movedSinceIssue))
            {
                if (Math.Abs(err) <= _s.FinishZoneIn)
                {
                    await _machine.StopAsync().ConfigureAwait(false);
                    return MoveOutcome.Near;      // too close for a drive — caller finishes with slow jog
                }
                // LS1+ drops a drive after ~1 s — re-issue it (MoveToAsync stops + settles first).
                idleResumes = Math.Abs(r.PositionIn - posAtResume) < 0.0005 ? idleResumes + 1 : 0;
                if (idleResumes >= 2)
                {
                    await _machine.StopAsync().ConfigureAwait(false);
                    throw new InvalidOperationException(NotRespondingMsg);
                }
                posAtResume = r.PositionIn;
                movedSinceIssue = false;
                await WaitStoppedAsync(ct).ConfigureAwait(false);
                await _machine.MoveToAsync(target, speedInPerMin, ct).ConfigureAwait(false);
                sinceProgress.Restart();
                grace.Restart();
                speedMon = new SpeedMonitor();
            }
            else if (jog && sinceProgress.ElapsedMilliseconds > 2500)
            {
                await _machine.StopAsync().ConfigureAwait(false);
                return MoveOutcome.Stalled;
            }

            if (sw.Elapsed > timeout)
            {
                await _machine.StopAsync().ConfigureAwait(false);
                throw new TimeoutException($"Move to {target:0.000} in timed out.");
            }
            await Task.Delay(5, ct).ConfigureAwait(false);
        }
    }

    /// <summary>
    /// Measures crosshead speed from the position stream. If the frame moves much faster than
    /// commanded (e.g. a speed-unit mismatch), stop immediately.
    /// </summary>
    private sealed class SpeedMonitor
    {
        private readonly Queue<(DateTime t, double p)> _q = new();
        public double? Last { get; private set; }
        public double? MmPerMin(MachineReading r)
        {
            if (_q.Count == 0 || _q.Last().t != r.Time) _q.Enqueue((r.Time, r.PositionIn));
            while (_q.Count > 2 && (r.Time - _q.Peek().t).TotalSeconds > 0.6) _q.Dequeue();
            var first = _q.Peek();
            double dt = (r.Time - first.t).TotalSeconds;
            if (dt < 0.25) return null;
            Last = Math.Abs(r.PositionIn - first.p) / dt * 60.0 * TestSettings.MM_PER_IN;
            return Last;
        }
    }

    /// <param name="commandedInPerMin">null = jog mode (checked against MaxAutoSpeedMmMin instead).</param>
    private void CheckSpeed(SpeedMonitor mon, MachineReading r, double? commandedInPerMin)
    {
        if (mon.MmPerMin(r) is not double actual) return;
        if (commandedInPerMin is double c)
        {
            double cmd = c * TestSettings.MM_PER_IN;
            if (actual > cmd * 2.0 + 3.0)
            {
                _ = _machine.StopAsync();
                throw new InvalidOperationException(
                    $"SPEED WATCHDOG: crosshead moving {actual:0} mm/min but {cmd:0.#} mm/min was commanded. Stopped. " +
                    "Switch MOTION to 'Slow jog' on the Setup page.");
            }
        }
        else if (actual > _s.MaxAutoSpeedMmMin)
        {
            _ = _machine.StopAsync();
            throw new InvalidOperationException(
                $"SPEED WATCHDOG: slow jog is moving {actual:0} mm/min, above the {_s.MaxAutoSpeedMmMin:0} mm/min limit. Stopped.");
        }
    }

    /// <summary>Undo the force zero taken at the preload, so force reads against the ZERO (spring-free) tare again.</summary>
    private async Task RestoreForceZeroAsync(CancellationToken ct)
    {
        if (_preloadOffsetN == 0) return;
        await _machine.ShiftForceZeroAsync(_preloadOffsetN).ConfigureAwait(false);
        _preloadOffsetN = 0;
        await NextReadingAsync(ct).ConfigureAwait(false);
    }

    /// <summary>Wait (max 2 s) until the crosshead has stopped moving.</summary>
    private async Task WaitStoppedAsync(CancellationToken ct)
    {
        var sw = Stopwatch.StartNew();
        await Task.Delay(150, ct).ConfigureAwait(false);
        while (FreshLatest().IsMoving && sw.ElapsedMilliseconds < 2000)
            await Task.Delay(20, ct).ConfigureAwait(false);
    }

    /// <summary>Wait until a reading newer than 'now' has arrived (so offsets just applied are reflected).</summary>
    private async Task NextReadingAsync(CancellationToken ct)
    {
        var since = DateTime.UtcNow;
        var sw = Stopwatch.StartNew();
        while (Latest.Time <= since)
        {
            if (sw.Elapsed.TotalSeconds > _s.StaleReadingSeconds)
                throw new InvalidOperationException("No readings from the machine — check the readings port (COM4).");
            await Task.Delay(5, ct).ConfigureAwait(false);
        }
    }

    /// <summary>Latest reading, or throw if the readings stream has gone quiet.</summary>
    private MachineReading FreshLatest()
    {
        var r = Latest;
        if ((DateTime.UtcNow - r.Time).TotalSeconds > _s.StaleReadingSeconds)
            throw new InvalidOperationException("No readings from the machine — check the readings port (COM4).");
        return r;
    }

    /// <summary>Average force and position over a window of distinct readings.</summary>
    private async Task<(double avgN, double avgPosIn, int n)> SampleAsync(TimeSpan window, CancellationToken ct)
    {
        var sw = Stopwatch.StartNew();
        double sumF = 0, sumP = 0; int n = 0; DateTime last = default;
        while (sw.Elapsed < window || n == 0)
        {
            ct.ThrowIfCancellationRequested();
            if (n == 0 && sw.Elapsed > window + TimeSpan.FromSeconds(2))
                throw new InvalidOperationException("No force readings received while sampling.");
            var r = Latest;
            if (r.Time != last) { sumF += r.ForceN; sumP += r.PositionIn; n++; last = r.Time; }
            await Task.Delay(5, ct).ConfigureAwait(false);
        }
        return (sumF / n, sumP / n, n);
    }

    private async Task PollLoopAsync(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            try
            {
                var r = await _machine.ReadAsync(ct).ConfigureAwait(false);
                MachineReading prev;
                lock (_readGate) { prev = _latest; _latest = r; }
                if (r.Time != prev.Time && prev.Time != default)
                {
                    double dt = (r.Time - prev.Time).TotalSeconds;
                    if (dt > 0) ReadingsHz = ReadingsHz * 0.9 + (1.0 / dt) * 0.1;
                }
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
