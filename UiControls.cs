using System.Drawing.Drawing2D;
using System.Text;

namespace LsSpringTester;

/// <summary>Palette + owner-drawn controls shared with the AQM calibrator's look.</summary>
internal static class Ui
{
    public static readonly Color BG = Color.FromArgb(18, 19, 22);
    public static readonly Color SIDEBAR = Color.FromArgb(23, 24, 28);
    public static readonly Color CARD = Color.FromArgb(30, 32, 37);
    public static readonly Color CARD_HI = Color.FromArgb(37, 39, 45);
    public static readonly Color HERO = Color.FromArgb(26, 28, 33);
    public static readonly Color BORDER = Color.FromArgb(48, 51, 58);
    public static readonly Color BORDER_HI = Color.FromArgb(70, 74, 84);
    public static readonly Color INK = Color.FromArgb(240, 242, 245);
    public static readonly Color TEXT_MID = Color.FromArgb(150, 156, 166);
    public static readonly Color TEXT_DIM = Color.FromArgb(96, 102, 112);
    public static readonly Color WHITE = Color.FromArgb(245, 247, 250);
    public static readonly Color ACCENT_FILL = Color.FromArgb(238, 240, 244);
    public static readonly Color DARK_TEXT = Color.FromArgb(18, 19, 22);
    public static readonly Color AMBER = Color.FromArgb(210, 178, 110);
    public static readonly Color GOOD = Color.FromArgb(150, 200, 170);
    public static readonly Color FAULT = Color.FromArgb(224, 122, 122);
    public static readonly Color GOOD_FILL = Color.FromArgb(32, 50, 40);
    public static readonly Color GOOD_EDGE = Color.FromArgb(70, 110, 85);
    public static readonly Color FAULT_FILL = Color.FromArgb(62, 32, 32);
    public static readonly Color FAULT_EDGE = Color.FromArgb(120, 62, 62);
    public static readonly Color STOP_FILL = Color.FromArgb(58, 34, 34);
    public static readonly Color STOP_EDGE = Color.FromArgb(110, 60, 60);
    public static readonly Color RETURN_FILL = Color.FromArgb(150, 46, 46);
    public static readonly Color RETURN_EDGE = Color.FromArgb(196, 84, 84);

    public static Color Blend(Color a, Color b, float t) => Color.FromArgb(
        Math.Clamp((int)(a.R + (b.R - a.R) * t), 0, 255),
        Math.Clamp((int)(a.G + (b.G - a.G) * t), 0, 255),
        Math.Clamp((int)(a.B + (b.B - a.B) * t), 0, 255));

    public static string Spaced(string s)
    {
        var sb = new StringBuilder();
        foreach (char c in s) { sb.Append(c); sb.Append('\u2009'); }
        return sb.ToString().TrimEnd();
    }

    public static GraphicsPath RoundRect(Rectangle r, int radius)
    {
        int d = radius * 2;
        var p = new GraphicsPath();
        if (d <= 0) { p.AddRectangle(r); p.CloseFigure(); return p; }
        p.AddArc(r.X, r.Y, d, d, 180, 90);
        p.AddArc(r.Right - d, r.Y, d, d, 270, 90);
        p.AddArc(r.Right - d, r.Bottom - d, d, d, 0, 90);
        p.AddArc(r.X, r.Bottom - d, d, d, 90, 90);
        p.CloseFigure();
        return p;
    }
}

/// <summary>Anti-aliased rounded card with hairline border.</summary>
internal class RoundedPanel : Panel
{
    public int Radius = 12;
    public Color Fill = Ui.CARD;
    public Color Edge = Ui.BORDER;

    public RoundedPanel()
    {
        SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint
               | ControlStyles.UserPaint | ControlStyles.ResizeRedraw, true);
        BackColor = Ui.BG;
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.Clear(BackColor);
        g.SmoothingMode = SmoothingMode.AntiAlias;
        var r = new Rectangle(0, 0, Width - 1, Height - 1);
        using var path = Ui.RoundRect(r, Radius);
        using (var b = new SolidBrush(Fill)) g.FillPath(b, path);
        using (var p = new Pen(Edge, 1)) g.DrawPath(p, path);
        base.OnPaint(e);
    }

    public void SetColors(Color fill, Color edge)
    {
        Fill = fill;
        Edge = edge;
        Invalidate(true);
    }
}

/// <summary>Owner-drawn pill / soft-rect button.</summary>
internal class RoundedButton : Button
{
    public int Radius = 8;
    public Color FillNormal = Ui.CARD_HI;
    public Color FillHover = Ui.Blend(Ui.CARD_HI, Ui.WHITE, 0.06f);
    public Color FillDown = Ui.CARD;
    public Color EdgeColor = Ui.BORDER;
    public int EdgeSize = 1;
    private bool _hover, _down;

    public RoundedButton()
    {
        SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint, true);
        FlatStyle = FlatStyle.Flat;
        FlatAppearance.BorderSize = 0;
        Cursor = Cursors.Hand;
        ForeColor = Ui.INK;
        Font = new Font("Segoe UI", 9f, FontStyle.Bold);
        TabStop = false;
    }

    protected override void OnMouseEnter(EventArgs e) { _hover = true; Invalidate(); base.OnMouseEnter(e); }
    protected override void OnMouseLeave(EventArgs e) { _hover = false; _down = false; Invalidate(); base.OnMouseLeave(e); }
    protected override void OnMouseDown(MouseEventArgs e) { _down = true; Invalidate(); base.OnMouseDown(e); }
    protected override void OnMouseUp(MouseEventArgs e) { _down = false; Invalidate(); base.OnMouseUp(e); }
    protected override void OnEnabledChanged(EventArgs e) { Invalidate(); base.OnEnabledChanged(e); }

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.Clear(Parent is RoundedPanel rp ? rp.Fill : Parent?.BackColor ?? Ui.BG);
        g.SmoothingMode = SmoothingMode.AntiAlias;
        var r = new Rectangle(0, 0, Width - 1, Height - 1);
        using var path = Ui.RoundRect(r, Radius);
        Color fill = !Enabled ? Ui.Blend(FillNormal, Ui.CARD, 0.6f) : _down ? FillDown : _hover ? FillHover : FillNormal;
        using (var b = new SolidBrush(fill)) g.FillPath(b, path);
        if (EdgeSize > 0) using (var p = new Pen(EdgeColor, EdgeSize)) g.DrawPath(p, path);
        var fg = Enabled ? ForeColor : Ui.Blend(ForeColor, fill, 0.6f);
        var flags = TextFormatFlags.VerticalCenter | TextFormatFlags.EndEllipsis;
        if (TextAlign is ContentAlignment.MiddleLeft or ContentAlignment.TopLeft or ContentAlignment.BottomLeft)
        {
            flags |= TextFormatFlags.Left;
            r.Inflate(-10, 0);
        }
        else flags |= TextFormatFlags.HorizontalCenter;
        TextRenderer.DrawText(g, Text, Font, r, fg, flags);
    }

    public void Style(Color fill, Color edge, Color fg, int edgeSize = 1)
    {
        FillNormal = fill;
        FillHover = Ui.Blend(fill, Ui.WHITE, 0.07f);
        FillDown = Ui.Blend(fill, Ui.BG, 0.25f);
        EdgeColor = edge;
        EdgeSize = edgeSize;
        ForeColor = fg;
        Invalidate();
    }
}

/// <summary>Flicker-free panel for the trend chart.</summary>
internal class BufferedPanel : Panel
{
    public BufferedPanel()
    {
        SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint
               | ControlStyles.UserPaint | ControlStyles.ResizeRedraw, true);
    }
}
