param(
    [string]$ExecutablePath = (Join-Path $PSScriptRoot '..\build\Release\AeroSysLab.exe'),
    [string]$OutputPath = (Join-Path $PSScriptRoot '..\artifacts\rocket_orbital_coast.png')
)

Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class GncCoastApi {
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hwnd, out RECT rect);
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr hwnd, IntPtr hdc, uint flags);
    [DllImport("user32.dll")] public static extern IntPtr GetDlgItem(IntPtr hwnd, int id);
    [DllImport("user32.dll")] public static extern IntPtr SendMessage(IntPtr hwnd, uint message, IntPtr wParam, IntPtr lParam);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr hwnd, int command);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hwnd);
}
'@

$process = Start-Process -FilePath $ExecutablePath `
    -WorkingDirectory (Split-Path -Parent $ExecutablePath) -PassThru
try {
    for ($attempt = 0; $attempt -lt 50 -and $process.MainWindowHandle -eq 0; $attempt++) {
        Start-Sleep -Milliseconds 100
        $process.Refresh()
    }
    if ($process.MainWindowHandle -eq 0) { throw 'Main window was not created.' }
    $window = $process.MainWindowHandle
    [void][GncCoastApi]::ShowWindow($window, 3)
    [void][GncCoastApi]::SetForegroundWindow($window)
    [void][GncCoastApi]::SendMessage($window, 0x0111, [IntPtr]101, [IntPtr]0)
    $playback = [GncCoastApi]::GetDlgItem($window, 503)
    [void][GncCoastApi]::SendMessage($playback, 0x014E, [IntPtr]9, [IntPtr]0)
    [void][GncCoastApi]::SendMessage($window, 0x0111, [IntPtr]110, [IntPtr]0)
    Start-Sleep -Seconds 2

    $rect = New-Object GncCoastApi+RECT
    [void][GncCoastApi]::GetWindowRect($window, [ref]$rect)
    $bitmap = New-Object System.Drawing.Bitmap($($rect.Right - $rect.Left), $($rect.Bottom - $rect.Top))
    $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
    $hdc = $graphics.GetHdc()
    try {
        if (-not [GncCoastApi]::PrintWindow($window, $hdc, 2)) {
            throw 'PrintWindow failed.'
        }
    }
    finally {
        $graphics.ReleaseHdc($hdc)
    }
    $bitmap.Save($OutputPath, [System.Drawing.Imaging.ImageFormat]::Png)
    $graphics.Dispose()
    $bitmap.Dispose()
}
finally {
    if (!$process.HasExited) { $process.CloseMainWindow() | Out-Null }
}
