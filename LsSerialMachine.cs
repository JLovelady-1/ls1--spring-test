using System.Globalization;
using System.IO.Ports;
using System.Text;

namespace LsSpringTester;

/// <summary>
/// LS1+ adapter, ported from the AQM calibrator (WinFormsApp1/Form1.cs):
///
///   CONTROL  port (COM5)  57600 8N1, RTS/DTR on, commands end in "\n"
///       CtlJogMachine(0)            stop
///       CtlJogMachine(1) / (2)      slow jog up / down  (machine times out → re-sent every 300 ms)
///       MtmRunDriveStage(1, 0, targetMm, rateMmPerSec, 0, 0, 0,0,0,0, false, 0)   drive to position (rate = mm/s)
///       Every drive is preceded by stop + 150 ms settle (stacked drives lock up the LS1+).
///
///   READINGS port (COM4)  9600 8N1, ASCII lines, tab-separated, streamed by the machine
///       4 \t 0 \t <position mm>
///       4 \t 1 \t <load N>
///       2 \t ... (9-field status line, not used here)
///
/// Tare/zero are done in SOFTWARE (offsets on the streamed values) so force can be tared
/// without touching position and vice versa; CtlZeroReadings is not used.
/// "Is moving" is derived from the position stream, since the status line isn't decoded.
/// </summary>
public sealed class LsSerialMachine : ILsMachine
{
    // ── Protocol constants (from the AQM app) ────────────────────────────────
    private const int CONTROL_BAUD = 57600;
    private const int READINGS_BAUD = 9600;
    private const string CMD_STOP = "CtlJogMachine(0)";
    private const string CMD_JOG_UP_SLOW = "CtlJogMachine(1)";
    private const string CMD_JOG_DOWN_SLOW = "CtlJogMachine(2)";
    private const string CMD_JOG_UP_FAST = "CtlJogMachine(3)";
    private const string CMD_JOG_DOWN_FAST = "CtlJogMachine(4)";
    private const int JOG_KEEPALIVE_MS = 300;
    // Harmless Lua call used as a heartbeat while a drive stage runs (CtlSetVariable is used by NEXYGEN itself).
    private const string CMD_DRIVE_KEEPALIVE = "CtlSetVariable('lsst_keepalive', 1)";
    private const int DRIVE_KEEPALIVE_MS = 250;
    private const int SETTLE_BEFORE_DRIVE_MS = 150;
    private const double MM_PER_IN = 25.4;

    // Firmware signature (Lloyd.MaterialTestMachineFirmware, Common/DriveStageLib.lua):
    //   MtmRunDriveStage(stageId, ds, limit, rate, speedType, speedDS, gaugeLength,
    //                    holdTime, rampUpTime, rampDownTime, <bool>, <int>)
    // ServiceUtility calls it as MtmRunDriveStage(1, Crosshead, 5, 40 / 60, 0, Crosshead, 0,0,0,0, false, 0)
    // → 'rate' is in mm per SECOND. We work in mm/min, so divide by 60.
    private const double SPEED_SCALE = 1.0 / 60.0;

    // ── Frame orientation — VERIFY ON FIRST RUN ──────────────────────────────
    // Assumes crosshead UP stretches the spring and the machine reports up as +mm.
    // If jogging "extend" compresses the spring, or position counts down while
    // extending, flip EXTEND_IS_UP.
    private const bool EXTEND_IS_UP = true;
    private const double POS_SIGN = EXTEND_IS_UP ? 1.0 : -1.0;
    private const double FORCE_SIGN = 1.0;          // AQM app: tension targets are +N

    // Motion detection from the position stream
    private const double MOVE_THRESHOLD_MM = 0.002;
    private const int MOVING_HOLD_MS = 300;

    private readonly string _controlPortName, _readingsPortName;
    private readonly object _writeGate = new();
    private readonly object _stateGate = new();

    private SerialPort? _ctl, _rd;
    private Thread? _rdThread;
    private volatile bool _rdRunning;
    private System.Threading.Timer? _jogTimer;   // explicit: WinForms also has a Timer
    private string? _jogCmd;           // guarded by _writeGate
    private bool _driveActive;         // guarded by _writeGate
    private System.Threading.Timer? _driveTimer;
    public bool KeepAliveDrives { get; set; } = true;
    private long _motionGen;           // bumped by every Stop; guarded by _writeGate

    // streamed state (guarded by _stateGate)
    private double _rawPosMm, _rawLoadN, _posZeroMm, _loadZeroN;
    private bool _havePos, _haveLoad;
    private DateTime _lastLineUtc;
    private double _motionRefMm;
    private DateTime _lastMoveUtc;

    public LsSerialMachine(string controlPort, string readingsPort)
    {
        _controlPortName = controlPort;
        _readingsPortName = readingsPort;
    }

    public bool IsConnected => _ctl?.IsOpen == true && _rd?.IsOpen == true;
    public event Action<string>? CommandSent;

    // ─────────────────────────── connection ───────────────────────────

    public async Task ConnectAsync(CancellationToken ct)
    {
        if (string.IsNullOrWhiteSpace(_controlPortName) || string.IsNullOrWhiteSpace(_readingsPortName))
            throw new InvalidOperationException("Select both the control (COM5) and readings (COM4) ports.");

        _ctl = new SerialPort(_controlPortName, CONTROL_BAUD, Parity.None, 8, StopBits.One)
        {
            ReadTimeout = 200,
            WriteTimeout = 500,
            RtsEnable = true,
            DtrEnable = true,
        };
        _ctl.Open();
        _ctl.DiscardInBuffer();
        _ctl.DiscardOutBuffer();

        _rd = new SerialPort(_readingsPortName, READINGS_BAUD, Parity.None, 8, StopBits.One)
        {
            Encoding = Encoding.ASCII,
            NewLine = "\n",
            ReadTimeout = 200,
        };
        _rd.Open();

        _rdRunning = true;
        _rdThread = new Thread(ReadingsThreadProc) { IsBackground = true, Name = "LS readings", Priority = ThreadPriority.AboveNormal };
        _rdThread.Start();

        _jogTimer = new System.Threading.Timer(JogKeepAlive, null, Timeout.Infinite, Timeout.Infinite);
        _driveTimer = new System.Threading.Timer(DriveKeepAlive, null, Timeout.Infinite, Timeout.Infinite);

        // Make sure the machine is stopped and actually streaming before we hand control over.
        WriteNow(CMD_STOP);
        var deadline = DateTime.UtcNow.AddSeconds(3);
        while (DateTime.UtcNow < deadline)
        {
            lock (_stateGate) { if (_havePos && _haveLoad) return; }
            await Task.Delay(50, ct).ConfigureAwait(false);
        }
        throw new InvalidOperationException(
            $"No position/load data on {_readingsPortName} within 3 s — check the readings cable/port.");
    }

    public Task DisconnectAsync()
    {
        try { StopAsync(); } catch { }
        Shutdown();
        return Task.CompletedTask;
    }

    // ─────────────────────────── readings ───────────────────────────

    public Task<MachineReading> ReadAsync(CancellationToken ct)
    {
        lock (_stateGate)
        {
            bool moving = (DateTime.UtcNow - _lastMoveUtc).TotalMilliseconds < MOVING_HOLD_MS;
            return Task.FromResult(new MachineReading(
                FORCE_SIGN * (_rawLoadN - _loadZeroN),
                POS_SIGN * (_rawPosMm - _posZeroMm) / MM_PER_IN,
                moving,
                _lastLineUtc));
        }
    }

    public double HomePositionIn
    {
        get { lock (_stateGate) return POS_SIGN * (0.0 - _posZeroMm) / MM_PER_IN; }
    }

    public Task ZeroForceAsync(CancellationToken ct)
    {
        lock (_stateGate) _loadZeroN = _rawLoadN;
        return Task.CompletedTask;
    }

    public Task ShiftForceZeroAsync(double deltaN)
    {
        lock (_stateGate) _loadZeroN -= FORCE_SIGN * deltaN;
        return Task.CompletedTask;
    }

    public Task ZeroPositionAsync(CancellationToken ct)
    {
        lock (_stateGate) _posZeroMm = _rawPosMm;
        return Task.CompletedTask;
    }

    private void ReadingsThreadProc()
    {
        var sb = new StringBuilder();
        while (_rdRunning)
        {
            try
            {
                var port = _rd;
                if (port is not { IsOpen: true }) { Thread.Sleep(100); continue; }
                int avail = port.BytesToRead;
                if (avail <= 0) { Thread.Sleep(5); continue; }

                var buf = new byte[avail];
                int n = port.Read(buf, 0, avail);
                sb.Append(Encoding.ASCII.GetString(buf, 0, n));

                string all = sb.ToString();
                int nl;
                while ((nl = all.IndexOf('\n')) >= 0)
                {
                    string line = all.Substring(0, nl).Trim();
                    all = all.Substring(nl + 1);
                    if (line.Length > 0) ProcessLine(line);
                }
                sb.Clear();
                sb.Append(all);
                if (sb.Length > 512) sb.Clear();
            }
            catch (TimeoutException) { }
            catch (Exception)
            {
                if (!_rdRunning) break;
                Thread.Sleep(200);
                try { _rd?.DiscardInBuffer(); } catch { }
            }
        }
    }

    private void ProcessLine(string line)
    {
        var parts = line.Split('\t');
        if (parts.Length < 3 || parts[0] != "4") return;
        if (!double.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out double v)) return;

        var now = DateTime.UtcNow;
        lock (_stateGate)
        {
            if (parts[1] == "0")
            {
                _rawPosMm = v;
                _havePos = true;
                if (Math.Abs(v - _motionRefMm) > MOVE_THRESHOLD_MM) { _motionRefMm = v; _lastMoveUtc = now; }
                _lastLineUtc = now;
            }
            else if (parts[1] == "1")
            {
                _rawLoadN = v;
                _haveLoad = true;
                _lastLineUtc = now;
            }
        }
    }

    // ─────────────────────────── motion ───────────────────────────

    /// <summary>PRIORITY. Synchronous; cancels jog keep-alive and any pending drive.</summary>
    public Task StopAsync()
    {
        lock (_writeGate)
        {
            _jogCmd = null;
            _driveActive = false;
            _motionGen++;
            WriteLocked(CMD_STOP);
        }
        return Task.CompletedTask;
    }

    public Task JogAsync(JogDirection dir, bool fast, CancellationToken ct)
    {
        bool up = (dir == JogDirection.Extend) == EXTEND_IS_UP;
        lock (_writeGate)
        {
            _driveActive = false;
            _jogCmd = up ? (fast ? CMD_JOG_UP_FAST : CMD_JOG_UP_SLOW)
                         : (fast ? CMD_JOG_DOWN_FAST : CMD_JOG_DOWN_SLOW);
            WriteLocked(_jogCmd);
            CommandSent?.Invoke($"TX {_jogCmd}  (held, repeated every {JOG_KEEPALIVE_MS} ms)");
        }
        _jogTimer?.Change(JOG_KEEPALIVE_MS, JOG_KEEPALIVE_MS);
        return Task.CompletedTask;
    }

    // call with _writeGate held
    private void StartDriveKeepAlive()
    {
        _driveActive = KeepAliveDrives;
        if (_driveActive) _driveTimer?.Change(DRIVE_KEEPALIVE_MS, DRIVE_KEEPALIVE_MS);
    }

    private void DriveKeepAlive(object? _)
    {
        lock (_writeGate)
        {
            if (!_driveActive) { _driveTimer?.Change(Timeout.Infinite, Timeout.Infinite); return; }
            try { WriteLocked(CMD_DRIVE_KEEPALIVE, log: false); } catch { }
        }
    }

    private void JogKeepAlive(object? _)
    {
        lock (_writeGate)
        {
            if (_jogCmd == null) { _jogTimer?.Change(Timeout.Infinite, Timeout.Infinite); return; }
            WriteLocked(_jogCmd);
        }
    }

    public async Task MoveToAsync(double positionIn, double speedInPerMin, CancellationToken ct)
    {
        double targetMm;
        lock (_stateGate) targetMm = _posZeroMm + POS_SIGN * positionIn * MM_PER_IN;
        double speedMm = Math.Max(0.01, speedInPerMin * MM_PER_IN) * SPEED_SCALE;

        long gen;
        lock (_writeGate)
        {
            _jogCmd = null;
            _driveActive = false;
            _motionGen++;
            gen = _motionGen;
            WriteLocked(CMD_STOP);                       // never stack drive commands
        }

        await Task.Delay(SETTLE_BEFORE_DRIVE_MS, ct).ConfigureAwait(false);

        lock (_writeGate)
        {
            // A STOP issued during the settle wins — don't send the drive.
            if (gen != _motionGen || ct.IsCancellationRequested) return;
            WriteLocked(DriveCmd(0, targetMm, speedMm));
            StartDriveKeepAlive();
        }
    }

    public async Task DriveToLoadAsync(double loadN, double speedInPerMin, CancellationToken ct)
    {
        double targetRawN;
        lock (_stateGate) targetRawN = _loadZeroN + FORCE_SIGN * loadN;     // tare is software-side
        double speed = Math.Max(0.01, speedInPerMin * MM_PER_IN) * SPEED_SCALE;

        long gen;
        lock (_writeGate)
        {
            _jogCmd = null;
            _driveActive = false;
            _motionGen++;
            gen = _motionGen;
            WriteLocked(CMD_STOP);
        }
        await Task.Delay(SETTLE_BEFORE_DRIVE_MS, ct).ConfigureAwait(false);
        lock (_writeGate)
        {
            if (gen != _motionGen || ct.IsCancellationRequested) return;
            WriteLocked(DriveCmd(1, targetRawN, speed));
            StartDriveKeepAlive();
        }
    }

    // ds (limit data source): 0 = crosshead position (mm), 1 = load (N) — same as the AQM app.
    // speedType 0, speedDS 0 (crosshead), no gauge length / hold / ramps — matches the
    // ServiceUtility production-test call.
    // Firmware (Common/DriveStageLib.lua):
    //   MtmRunDriveStageEx(stageId, ds, limit, rate, speedType, speedDS, gaugeLength, holdTime,
    //                      rampUpTime, rampDownTime, isFollowOn, requestId, detectors)
    //   • requestId ~= 0 → the frame ACKs this long-running call immediately (NEXYGEN always passes one).
    //     With requestId = 0 there is no ack, and the drive was being cut off after ~1 s.
    //   • isFollowOn = false → MtmRunStage (runs to the limit); true → start + return immediately.
    // A request id makes the drive a long BLOCKING call on the frame; interrupting one mid-move can leave the
    // frame ignoring all further commands (seen 9/29). Keep 0 and use the keep-alive for smooth motion.
    private const bool USE_REQUEST_ID = false;
    private const bool USE_FOLLOW_ON = false;
    private int _requestId;

    private string DriveCmd(int channel, double target, double rateMmPerSec)
    {
        string t = target.ToString("F4", CultureInfo.InvariantCulture);
        string s = rateMmPerSec.ToString("F6", CultureInfo.InvariantCulture);
        _requestId = !USE_REQUEST_ID ? 0 : (_requestId >= 30000 ? 1 : _requestId + 1);
        string followOn = USE_FOLLOW_ON ? "true" : "false";
        return $"MtmRunDriveStage(1, {channel}, {t}, {s}, 0, 0, 0, 0, 0, 0, {followOn}, {_requestId})";
    }

    // ─────────────────────────── transport ───────────────────────────

    private void WriteNow(string cmd)
    {
        lock (_writeGate) WriteLocked(cmd);
    }

    private void WriteLocked(string cmd, bool log = true)
    {
        var port = _ctl;
        if (port is not { IsOpen: true }) return;
        port.DiscardInBuffer();                          // replies aren't used
        port.Write(cmd + "\n");
        if (log && (!ReferenceEquals(cmd, _jogCmd) || cmd == CMD_STOP)) CommandSent?.Invoke("TX " + cmd);
    }

    private void Shutdown()
    {
        _rdRunning = false;
        try { _jogTimer?.Dispose(); } catch { }
        _jogTimer = null;
        try { _driveTimer?.Dispose(); } catch { }
        _driveTimer = null;
        try { _rdThread?.Join(500); } catch { }
        try { _ctl?.Close(); _ctl?.Dispose(); } catch { }
        try { _rd?.Close(); _rd?.Dispose(); } catch { }
        _ctl = null;
        _rd = null;
    }

    public void Dispose()
    {
        try { StopAsync(); } catch { }
        Shutdown();
    }
}
