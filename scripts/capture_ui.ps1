param(
    [string]$ExecutablePath = (Join-Path $PSScriptRoot '..\build\Release\AeroSysLab.exe'),
    [string]$OutputDirectory = (Join-Path $PSScriptRoot '..\artifacts')
)

Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class GncWindowApi {
    [StructLayout(LayoutKind.Sequential)]
    public struct RECT { public int Left, Top, Right, Bottom; }
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hwnd, out RECT rect);
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr hwnd, IntPtr hdc, uint flags);
    [DllImport("user32.dll")] public static extern IntPtr SendMessage(IntPtr hwnd, uint message, IntPtr wParam, IntPtr lParam);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr hwnd, int command);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hwnd);
}
'@

function Save-WindowCapture {
    param([IntPtr]$WindowHandle, [string]$Path)
    $rect = New-Object GncWindowApi+RECT
    [void][GncWindowApi]::GetWindowRect($WindowHandle, [ref]$rect)
    $width = $rect.Right - $rect.Left
    $height = $rect.Bottom - $rect.Top
    $bitmap = New-Object System.Drawing.Bitmap($width, $height)
    $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
    $hdc = $graphics.GetHdc()
    try {
        if (-not [GncWindowApi]::PrintWindow($WindowHandle, $hdc, 2)) {
            throw 'PrintWindow failed.'
        }
    }
    finally {
        $graphics.ReleaseHdc($hdc)
    }
    $bitmap.Save($Path, [System.Drawing.Imaging.ImageFormat]::Png)
    $graphics.Dispose()
    $bitmap.Dispose()
}

function Pack-ClientPoint {
    param([int]$X, [int]$Y)
    return [IntPtr](([int64]$Y -shl 16) -bor ($X -band 0xffff))
}

New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$process = Start-Process -FilePath $ExecutablePath `
    -WorkingDirectory (Split-Path -Parent $ExecutablePath) -PassThru
try {
    for ($attempt = 0; $attempt -lt 50 -and $process.MainWindowHandle -eq 0; $attempt++) {
        Start-Sleep -Milliseconds 100
        $process.Refresh()
    }
    if ($process.MainWindowHandle -eq 0) { throw 'Main window was not created.' }
    [void][GncWindowApi]::ShowWindow($process.MainWindowHandle, 3)
    [void][GncWindowApi]::SetForegroundWindow($process.MainWindowHandle)
    Start-Sleep -Milliseconds 800
    Save-WindowCapture $process.MainWindowHandle (Join-Path $OutputDirectory 'satellite_ready.png')

    [void][GncWindowApi]::SendMessage($process.MainWindowHandle, 0x0111, [IntPtr]110, [IntPtr]0)
    Start-Sleep -Seconds 3
    Save-WindowCapture $process.MainWindowHandle (Join-Path $OutputDirectory 'satellite_running.png')

    # Exercise the real 3D camera path: rotate from one client point to another.
    [void][GncWindowApi]::SendMessage($process.MainWindowHandle, 0x0201, [IntPtr]1, (Pack-ClientPoint 380 360))
    [void][GncWindowApi]::SendMessage($process.MainWindowHandle, 0x0200, [IntPtr]1, (Pack-ClientPoint 500 420))
    [void][GncWindowApi]::SendMessage($process.MainWindowHandle, 0x0202, [IntPtr]0, (Pack-ClientPoint 500 420))
    Start-Sleep -Milliseconds 300
    Save-WindowCapture $process.MainWindowHandle (Join-Path $OutputDirectory 'satellite_rotated.png')

    [void][GncWindowApi]::SendMessage($process.MainWindowHandle, 0x0111, [IntPtr]111, [IntPtr]0)
    Start-Sleep -Milliseconds 700
    Save-WindowCapture $process.MainWindowHandle (Join-Path $OutputDirectory 'satellite_paused.png')
    [void][GncWindowApi]::SendMessage($process.MainWindowHandle, 0x0111, [IntPtr]111, [IntPtr]0)
    Start-Sleep -Milliseconds 500

    [void][GncWindowApi]::SendMessage($process.MainWindowHandle, 0x0111, [IntPtr]101, [IntPtr]0)
    [void][GncWindowApi]::SendMessage($process.MainWindowHandle, 0x0111, [IntPtr]110, [IntPtr]0)
    Start-Sleep -Seconds 3
    Save-WindowCapture $process.MainWindowHandle (Join-Path $OutputDirectory 'rocket_running.png')
}
finally {
    if (!$process.HasExited) { $process.CloseMainWindow() | Out-Null }
}
