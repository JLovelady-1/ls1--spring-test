namespace LsSpringTester;

/// <summary>One coherent snapshot from the machine. Force + = tension, Position + = extension.</summary>
public readonly record struct MachineReading(double ForceN, double PositionIn, bool IsMoving, DateTime Time);

public enum JogDirection { Extend, Retract }

/// <summary>
/// Everything the tester needs from the LS frame. Units are fixed here (N and inches,
/// + position = spring extension) so the adapter does all conversion/sign handling.
///
/// Rules for implementers:
///  • StopAsync must be safe from ANY thread at ANY time, must not wait behind other
///    traffic, and must complete synchronously (return Task.CompletedTask).
///  • JogAsync / MoveToAsync START motion and return; the controller watches the
///    readings and sends Stop when the target is reached.
///  • MoveToAsync must stop any current motion and settle before issuing the new
///    drive (stacked LS1+ drive commands lock up the controller).
/// </summary>
public interface ILsMachine : IDisposable
{
    bool IsConnected { get; }
    Task ConnectAsync(CancellationToken ct);
    Task DisconnectAsync();

    Task<MachineReading> ReadAsync(CancellationToken ct);

    Task StopAsync();
    /// <summary>Continuous jog at the machine's own slow or fast jog speed until StopAsync.</summary>
    Task JogAsync(JogDirection dir, bool fast, CancellationToken ct);
    Task MoveToAsync(double positionIn, double speedInPerMin, CancellationToken ct);

    /// <summary>Machine-side drive in LOAD mode: the frame itself stops at the target force (tared frame, + = tension).</summary>
    Task DriveToLoadAsync(double loadN, double speedInPerMin, CancellationToken ct);

    /// <summary>
    /// While a drive stage runs, periodically send a harmless command so the frame's host-comms timeout
    /// doesn't drop the drive after ~1 s (the same reason jog has to be re-sent every 300 ms).
    /// </summary>
    bool KeepAliveDrives { get; set; }

    /// <summary>Raised for every motion command actually sent (for the event log).</summary>
    event Action<string>? CommandSent;

    /// <summary>Machine home (machine position 0) expressed in the same frame as PositionIn.</summary>
    double HomePositionIn { get; }

    Task ZeroForceAsync(CancellationToken ct);
    Task ZeroPositionAsync(CancellationToken ct);

    /// <summary>Move the force zero so readings increase by deltaN (used to undo a zero taken at preload).</summary>
    Task ShiftForceZeroAsync(double deltaN);
}
