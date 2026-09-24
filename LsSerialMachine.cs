using System.Globalization;
using System.IO.Ports;

namespace LsSpringTester;

/// <summary>
/// ═══════════════════════════════════════════════════════════════════════════════
///  PORT FROM THE AQM APP: fill in the TODO command strings, baud rate, units and
///  the Parse* methods using the same LS1 protocol code you already have working.
///  Connect() refuses to open the port while any TODO remains, so nothing unknown
///  is ever sent to the frame. Use Simulator mode until this is filled in.
/// ═══════════════════════════════════════════════════════════════════════════════
/// Conventions the controller relies on:
///   • Force returned in NEWTONS, + = tension.
///   • Position returned in INCHES, + = spring extension (crosshead moving away).
///     If the frame reports mm or the opposite sign, convert here (MM_PER_IN / sign).
///   • Stop is written immediately (it only takes the byte-level write lock, never the
///     transaction lock), so it can't get stuck behind a poll.
/// </summary>
public sealed class LsSerialMachine : ILsMachine
{
    // ── TODO: copy from the AQM app ─────────────────────────────────────────────
    private const int BAUD = 9600;                               // TODO: confirm
    private const string NEWLINE = "\r";                         // TODO: confirm terminator
    private const string CMD_STOP = "TODO";
    private const string CMD_READ_FORCE = "TODO";
    private const string CMD_READ_POSITION = "TODO";
    private const string CMD_READ_STATUS = "TODO";               // must tell us moving / not moving
    private const string CMD_ZERO_FORCE = "TODO";
    private const string CMD_ZERO_POSITION = "TODO";
    private const string CMD_HOME = "TODO";
    private const string CMD_JOG_EXTEND_FMT = "TODO {0}";        // {0} = speed in machine units
    private const string CMD_JOG_RETRACT_FMT = "TODO {0}";
    private const string CMD_MOVE_TO_FMT = "TODO {0} {1}";       // {0} = position, {1} = speed
    private const bool   COMMANDS_RETURN_ACK = true;             // TODO: does every command reply?
    private const double MM_PER_IN = 25.4;
    private const double POSITION_SIGN = +1.0;                   // flip if extension reads negative
    private const double FORCE_SIGN = +1.0;                      // flip if tension reads negative
    // ────────────────────────────────────────────────────────────────────────────

    private readonly string _portName;
    private readonly SemaphoreSlim _txLock = new(1, 1);
    private readonly object _writeGate = new();
    private SerialPort? _port;

    public LsSerialMachine(string portName) => _portName = portName;

    public bool IsConnected => _port?.IsOpen == true;

    public Task ConnectAsync(CancellationToken ct)
    {
        var all = new[] { CMD_STOP, CMD_READ_FORCE, CMD_READ_POSITION, CMD_READ_STATUS, CMD_ZERO_FORCE,
                          CMD_ZERO_POSITION, CMD_HOME, CMD_JOG_EXTEND_FMT, CMD_JOG_RETRACT_FMT, CMD_MOVE_TO_FMT };
        if (all.Any(c => c.Contains("TODO")))
            throw new NotImplementedException(
                "LsSerialMachine has unfilled TODO commands — port the LS1 protocol from the AQM app first.");
        if (string.IsNullOrWhiteSpace(_portName))
            throw new InvalidOperationException("No COM port selected.");

        _port = new SerialPort(_portName, BAUD, Parity.None, 8, StopBits.One)
        {
            NewLine = NEWLINE,
            ReadTimeout = 500,
            WriteTimeout = 500,
        };
        _port.Open();
        return Task.CompletedTask;
    }

    public Task DisconnectAsync()
    {
        try { _port?.Close(); } catch { }
        return Task.CompletedTask;
    }

    public async Task<MachineReading> ReadAsync(CancellationToken ct)
    {
        string f = await QueryAsync(CMD_READ_FORCE, ct).ConfigureAwait(false);
        string p = await QueryAsync(CMD_READ_POSITION, ct).ConfigureAwait(false);
        string s = await QueryAsync(CMD_READ_STATUS, ct).ConfigureAwait(false);
        return new MachineReading(ParseForceN(f), ParsePositionIn(p), ParseIsMoving(s), DateTime.UtcNow);
    }

    // PRIORITY: bypasses the transaction lock. Synchronous by contract.
    public Task StopAsync()
    {
        var port = _port;
        if (port is { IsOpen: true })
        {
            lock (_writeGate) port.Write(CMD_STOP + port.NewLine);
        }
        // NOTE: if the frame ACKs the stop, a concurrent poll may read that ACK instead of its
        // own reply. Parse* should reject non-numeric replies (they throw → poll retries).
        return Task.CompletedTask;
    }

    public Task JogAsync(JogDirection dir, double speedInPerMin, CancellationToken ct)
    {
        string fmt = dir == JogDirection.Extend ? CMD_JOG_EXTEND_FMT : CMD_JOG_RETRACT_FMT;
        return SendAsync(string.Format(CultureInfo.InvariantCulture, fmt, SpeedToMachine(speedInPerMin)), ct);
    }

    public Task MoveToAsync(double positionIn, double speedInPerMin, CancellationToken ct) =>
        SendAsync(string.Format(CultureInfo.InvariantCulture, CMD_MOVE_TO_FMT,
            PositionToMachine(positionIn), SpeedToMachine(speedInPerMin)), ct);

    public Task HomeAsync(CancellationToken ct) => SendAsync(CMD_HOME, ct);
    public Task ZeroForceAsync(CancellationToken ct) => SendAsync(CMD_ZERO_FORCE, ct);
    public Task ZeroPositionAsync(CancellationToken ct) => SendAsync(CMD_ZERO_POSITION, ct);

    // ── unit conversion (TODO: match what the frame expects) ──
    private static double PositionToMachine(double inches) => POSITION_SIGN * inches * MM_PER_IN;   // mm
    private static double SpeedToMachine(double inPerMin) => inPerMin * MM_PER_IN;                   // mm/min

    // ── reply parsing (TODO: match the frame's reply format) ──
    private static double ParseForceN(string reply) =>
        FORCE_SIGN * double.Parse(reply.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture);

    private static double ParsePositionIn(string reply) =>
        POSITION_SIGN * double.Parse(reply.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture) / MM_PER_IN;

    private static bool ParseIsMoving(string reply) =>
        throw new NotImplementedException("TODO: decode the status reply into moving / stopped.");

    // ── transport ──
    private async Task<string> QueryAsync(string cmd, CancellationToken ct)
    {
        await _txLock.WaitAsync(ct).ConfigureAwait(false);
        try
        {
            return await Task.Run(() =>
            {
                var port = _port ?? throw new InvalidOperationException("Not connected.");
                lock (_writeGate) { port.DiscardInBuffer(); port.Write(cmd + port.NewLine); }
                return port.ReadLine();
            }, ct).ConfigureAwait(false);
        }
        finally { _txLock.Release(); }
    }

    private async Task SendAsync(string cmd, CancellationToken ct)
    {
        if (COMMANDS_RETURN_ACK) { await QueryAsync(cmd, ct).ConfigureAwait(false); return; }
        await _txLock.WaitAsync(ct).ConfigureAwait(false);
        try
        {
            var port = _port ?? throw new InvalidOperationException("Not connected.");
            lock (_writeGate) port.Write(cmd + port.NewLine);
        }
        finally { _txLock.Release(); }
    }

    public void Dispose()
    {
        try { StopAsync(); } catch { }
        try { _port?.Dispose(); } catch { }
    }
}
