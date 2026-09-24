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
///  • JogAsync / MoveToAsync / HomeAsync START motion and return; completion is detected
///    from MachineReading.IsMoving.
///  • MoveToAsync is a single machine-side positioning move (no host-side hunting).
/// </summary>
public interface ILsMachine : IDisposable
{
    bool IsConnected { get; }
    Task ConnectAsync(CancellationToken ct);
    Task DisconnectAsync();

    Task<MachineReading> ReadAsync(CancellationToken ct);

    Task StopAsync();
    Task JogAsync(JogDirection dir, double speedInPerMin, CancellationToken ct);
    Task MoveToAsync(double positionIn, double speedInPerMin, CancellationToken ct);
    Task HomeAsync(CancellationToken ct);

    Task ZeroForceAsync(CancellationToken ct);
    Task ZeroPositionAsync(CancellationToken ct);
}
