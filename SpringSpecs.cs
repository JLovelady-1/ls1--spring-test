namespace LsSpringTester;

public enum SpringFamily { Main, Trim }

/// <summary>
/// One row of an NC002679 (main) or NC002712 (trim) drawing table.
/// Pass/fail uses ONLY the load at TestLengthIn (2.67 in main / 2.75 in trim).
/// </summary>
public sealed record SpringSpec(
    string PartNumber,
    SpringFamily Family,
    string Capacity,
    double RateLbPerIn,
    double RateTolLbPerIn,
    double TestLengthIn,
    double LoadNomLbf,
    double LoadTolLbf,
    string Color1,
    string Color2)
{
    public const double N_TO_LBS = 0.224809;
    public const double N_PER_LBF = 4.448222;

    public double NominalFreeLengthIn =>
        Family == SpringFamily.Main ? SpringSpecs.MainFreeLengthIn : SpringSpecs.TrimFreeLengthIn;

    public double LoadMinLbf => LoadNomLbf - LoadTolLbf;
    public double LoadMaxLbf => LoadNomLbf + LoadTolLbf;

    public string ColorText => Color2 == "NONE" ? $"{Color1} / none" : $"{Color1} / {Color2}";

    public override string ToString() => Family == SpringFamily.Main
        ? $"{PartNumber}   ({Capacity})   [{ColorText}]"
        : $"{PartNumber}   ({RateLbPerIn:0.000} lb/in)   [{ColorText}]";
}

/// <summary>
/// ************************************************************************
/// VERIFY BEFORE USE: these tables were transcribed from photos of the
/// NC002679 and NC002712 drawings. Check every row against the controlled
/// drawing (especially load-at-test-length values and color codes).
/// NC002712-24 and -27 spring rates were inferred from their load columns.
/// ************************************************************************
/// </summary>
public static class SpringSpecs
{
    public const double MainFreeLengthIn = 2.25;   // NC002679 inspection length 2.25 ±.05
    public const double MainTestLengthIn = 2.67;   // "LOAD AT 2.67 IN" column
    public const double TrimFreeLengthIn = 1.75;   // NC002712 1.75 REF
    public const double TrimTestLengthIn = 2.75;   // "LOAD AT 2.75 IN" column

    private static SpringSpec M(string dash, string cap, double rate, double rtol, double load, double tol, string c1, string c2) =>
        new($"NC002679-{dash}", SpringFamily.Main, cap, rate, rtol, MainTestLengthIn, load, tol, c1, c2);

    private static SpringSpec T(string dash, double rate, double rtol, double load, double tol, string c1, string c2) =>
        new($"NC002712-{dash}", SpringFamily.Trim, "", rate, rtol, TrimTestLengthIn, load, tol, c1, c2);

    //                              dash  capacity         rate    ±      load@2.67  ±     colors
    public static readonly IReadOnlyList<SpringSpec> Main = new[]
    {
        M("01", "1 kg",           2.20,  0.16,   0.90, 0.07, "BLUE",   "NONE"),
        M("02", "5 lb",           5.87,  0.44,   2.44, 0.20, "RED",    "NONE"),
        M("03", "10 lb",         11.50,  0.86,   4.88, 0.39, "GREEN",  "NONE"),
        M("04", "20 lb",         23.33,  1.75,   9.80, 0.78, "ORANGE", "NONE"),
        M("05", "30 lb",         34.67,  2.60,  14.20, 1.18, "BLACK",  "NONE"),
        M("06", "50 lb",         56.53,  4.24,  24.41, 1.95, "PINK",   "NONE"),
        M("07", "75 lb",         85.23,  6.39,  36.62, 2.93, "BROWN",  "NONE"),
        M("08", "100 lb",       119.43,  8.95,  48.88, 3.90, "VIOLET", "NONE"),
        M("09", "10 N",           2.47,  0.19,   1.02, 0.07, "VIOLET", "VIOLET"),
        M("10", "2 kg, 20 N",     5.17,  0.39,   2.17, 0.18, "BROWN",  "BROWN"),
        M("11", "3 kg, 30 N",     7.77,  0.58,   3.26, 0.26, "PINK",   "PINK"),
        M("12", "5 kg, 50 N",    13.10,  0.98,   5.43, 0.43, "BLACK",  "BLACK"),
        M("13", "10 kg, 100 N",  25.87,  1.94,  10.87, 0.87, "ORANGE", "ORANGE"),
        M("14", "20 kg, 200 N",  51.77,  3.88,  21.74, 1.74, "GREEN",  "GREEN"),
        M("15", "30 kg, 300 N",  77.63,  5.82,  32.61, 2.61, "RED",    "RED"),
        M("16", "50 kg, 500 N", 129.50,  9.71,  53.35, 4.30, "BLUE",   "BLUE"),
        M("17", "2 lb",           2.10,  0.15,   0.83, 0.07, "GREEN",  "RED"),
    };

    //                              dash  rate    ±      load@2.75  ±      colors
    public static readonly IReadOnlyList<SpringSpec> Trim = new[]
    {
        T("01", 0.063, 0.004,  0.104, 0.008, "BLUE",   "NONE"),
        T("02", 0.070, 0.004,  0.116, 0.009, "RED",    "NONE"),
        T("03", 0.078, 0.005,  0.129, 0.010, "GREEN",  "NONE"),
        T("04", 0.085, 0.005,  0.143, 0.011, "ORANGE", "NONE"),
        T("05", 0.095, 0.006,  0.159, 0.012, "BLACK",  "NONE"),
        T("06", 0.108, 0.006,  0.177, 0.013, "PINK",   "NONE"),
        T("07", 0.120, 0.007,  0.197, 0.014, "BROWN",  "NONE"),
        T("08", 0.273, 0.016,  0.420, 0.030, "VIOLET", "NONE"),
        T("09", 0.303, 0.018,  0.467, 0.032, "BLUE",   "BLUE"),
        T("10", 0.338, 0.020,  0.519, 0.036, "RED",    "RED"),
        T("11", 0.375, 0.022,  0.577, 0.040, "GREEN",  "GREEN"),
        T("12", 0.418, 0.025,  0.641, 0.044, "ORANGE", "ORANGE"),
        T("13", 0.463, 0.028,  0.712, 0.048, "BLACK",  "BLACK"),
        T("14", 0.513, 0.031,  0.791, 0.055, "PINK",   "PINK"),
        T("15", 0.570, 0.034,  0.879, 0.061, "BROWN",  "BROWN"),
        T("16", 0.635, 0.038,  0.977, 0.068, "VIOLET", "VIOLET"),
        T("17", 0.725, 0.043,  1.090, 0.080, "BLUE",   "RED"),
        T("18", 0.800, 0.048,  1.210, 0.080, "BLUE",   "GREEN"),
        T("19", 0.875, 0.052,  1.340, 0.090, "BLUE",   "ORANGE"),
        T("20", 1.000, 0.060,  1.490, 0.100, "BLUE",   "BLACK"),
        T("21", 1.180, 0.070,  1.660, 0.120, "BLUE",   "PINK"),
        T("22", 1.380, 0.080,  1.840, 0.130, "BLUE",   "BROWN"),
        T("23", 1.730, 0.100,  2.050, 0.150, "BLUE",   "VIOLET"),
        T("24", 1.900, 0.110,  2.270, 0.160, "RED",    "GREEN"),
        T("25", 2.150, 0.120,  2.530, 0.180, "RED",    "ORANGE"),
        T("26", 2.380, 0.140,  2.810, 0.200, "RED",    "BLACK"),
        T("27", 2.650, 0.150,  3.130, 0.220, "RED",    "PINK"),
        T("28", 2.930, 0.170,  3.470, 0.240, "RED",    "BROWN"),
        T("29", 3.530, 0.210,  3.860, 0.270, "RED",    "VIOLET"),
        T("30", 3.880, 0.230,  4.290, 0.300, "GREEN",  "ORANGE"),
        T("31", 4.150, 0.240,  4.760, 0.330, "GREEN",  "BLACK"),
        T("32", 4.480, 0.260,  5.290, 0.370, "GREEN",  "PINK"),
        T("33", 4.960, 0.290,  5.870, 0.410, "GREEN",  "BROWN"),
        T("34", 5.830, 0.340,  6.530, 0.460, "GREEN",  "VIOLET"),
        T("35", 7.030, 0.420,  7.260, 0.510, "ORANGE", "BLACK"),
        T("36", 8.000, 0.460,  8.060, 0.560, "ORANGE", "PINK"),
        T("37", 8.900, 0.530,  8.960, 0.630, "ORANGE", "BROWN"),
        T("38", 9.900, 0.590,  9.960, 0.700, "ORANGE", "VIOLET"),
        T("39", 10.980, 0.660, 11.060, 0.770, "BLACK", "PINK"),
        T("40", 0.048, 0.002,  0.077, 0.006, "BLACK",  "BROWN"),
        T("41", 0.053, 0.003,  0.085, 0.006, "BLACK",  "VIOLET"),
        T("42", 0.058, 0.003,  0.094, 0.007, "PINK",   "BROWN"),
        T("43", 0.135, 0.008,  0.219, 0.016, "PINK",   "VIOLET"),
        T("44", 0.150, 0.010,  0.243, 0.019, "BROWN",  "VIOLET"),
    };
}
