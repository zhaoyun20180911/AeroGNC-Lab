param(
    [string]$ExecutablePath = (Join-Path $PSScriptRoot '..\build\Release\AeroSysLab.exe'),
    [string]$OutputDirectory = (Join-Path $PSScriptRoot '..\artifacts')
)

Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
using System.Text;
public static class GncSettingsApi {
    public delegate bool EnumWindowsProc(IntPtr hwnd, IntPtr lParam);
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
    [DllImport("user32.dll")] public static extern bool EnumWindows(EnumWindowsProc callback, IntPtr lParam);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr hwnd);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr hwnd, StringBuilder text, int count);
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hwnd, out RECT rect);
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr hwnd, IntPtr hdc, uint flags);
    [DllImport("user32.dll")] public static extern IntPtr GetDlgItem(IntPtr hwnd, int id);
    [DllImport("user32.dll")] public static extern IntPtr SendMessage(IntPtr hwnd, uint message, IntPtr wParam, IntPtr lParam);
    [DllImport("user32.dll")] public static extern bool PostMessage(IntPtr hwnd, uint message, IntPtr wParam, IntPtr lParam);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr hwnd, int command);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hwnd);
}
'@

function Find-SettingsWindow {
    for ($attempt = 0; $attempt -lt 40; $attempt++) {
        $script:foundWindow = [IntPtr]::Zero
        $callback = [GncSettingsApi+EnumWindowsProc]{
            param([IntPtr]$hwnd, [IntPtr]$lParam)
            if ([GncSettingsApi]::IsWindowVisible($hwnd)) {
                $title = New-Object System.Text.StringBuilder 256
                [void][GncSettingsApi]::GetWindowText($hwnd, $title, $title.Capacity)
                $windowTitle = $title.ToString()
                if ($windowTitle -like 'AeroSys Lab v*' -and
                    ($windowTitle -like '*Settings*' -or $windowTitle -like '*Guidance*' -or
                     $windowTitle -like '*Orbit Control*' -or $windowTitle -like '*设置*')) {
                    $script:foundWindow = $hwnd
                    return $false
                }
            }
            return $true
        }
        [void][GncSettingsApi]::EnumWindows($callback, [IntPtr]::Zero)
        if ($script:foundWindow -ne [IntPtr]::Zero) { return $script:foundWindow }
        Start-Sleep -Milliseconds 100
    }
    throw 'Settings window was not created.'
}

function Save-WindowCapture {
    param([IntPtr]$WindowHandle, [string]$Path)
    $rect = New-Object GncSettingsApi+RECT
    [void][GncSettingsApi]::GetWindowRect($WindowHandle, [ref]$rect)
    $bitmap = New-Object System.Drawing.Bitmap($($rect.Right - $rect.Left), $($rect.Bottom - $rect.Top))
    $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
    $hdc = $graphics.GetHdc()
    try {
        if (-not [GncSettingsApi]::PrintWindow($WindowHandle, $hdc, 2)) {
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

New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$process = Start-Process -FilePath $ExecutablePath `
    -WorkingDirectory (Split-Path -Parent $ExecutablePath) -PassThru
try {
    for ($attempt = 0; $attempt -lt 50 -and $process.MainWindowHandle -eq 0; $attempt++) {
        Start-Sleep -Milliseconds 100
        $process.Refresh()
    }
    if ($process.MainWindowHandle -eq 0) { throw 'Main window was not created.' }
    [void][GncSettingsApi]::ShowWindow($process.MainWindowHandle, 3)

    foreach ($entry in @(
        @{ Id = 104; Name = 'control_settings.png' },
        @{ Id = 108; Name = 'satellite_orbit_control_settings.png' },
        @{ Id = 105; Name = 'satellite_disturbance_settings.png' },
        @{ Id = 106; Name = 'perturbation_settings.png' }
    )) {
        [void][GncSettingsApi]::PostMessage($process.MainWindowHandle, 0x0111, [IntPtr]$entry.Id, [IntPtr]0)
        $dialog = Find-SettingsWindow
        [void][GncSettingsApi]::SetForegroundWindow($dialog)
        Start-Sleep -Milliseconds 500
        Save-WindowCapture $dialog (Join-Path $OutputDirectory $entry.Name)
        if ($entry.Id -eq 104) {
            $customMode = [GncSettingsApi]::GetDlgItem($dialog, 103)
            [void][GncSettingsApi]::SendMessage($customMode, 0x00F5, [IntPtr]0, [IntPtr]0)
            Start-Sleep -Milliseconds 300
            Save-WindowCapture $dialog (Join-Path $OutputDirectory 'control_settings_custom.png')
        }
        [void][GncSettingsApi]::PostMessage($dialog, 0x0010, [IntPtr]0, [IntPtr]0)
        Start-Sleep -Milliseconds 250
    }
    [void][GncSettingsApi]::PostMessage($process.MainWindowHandle, 0x0111, [IntPtr]101, [IntPtr]0)
    Start-Sleep -Milliseconds 300
    foreach ($entry in @(
        @{ Id = 107; Name = 'rocket_guidance_settings.png' },
        @{ Id = 105; Name = 'rocket_disturbance_settings.png' }
    )) {
        [void][GncSettingsApi]::PostMessage($process.MainWindowHandle, 0x0111, [IntPtr]$entry.Id, [IntPtr]0)
        $rocketDialog = Find-SettingsWindow
        [void][GncSettingsApi]::SetForegroundWindow($rocketDialog)
        Start-Sleep -Milliseconds 500
        Save-WindowCapture $rocketDialog (Join-Path $OutputDirectory $entry.Name)
        [void][GncSettingsApi]::PostMessage($rocketDialog, 0x0010, [IntPtr]0, [IntPtr]0)
        Start-Sleep -Milliseconds 250
    }
}
finally {
    if (!$process.HasExited) { $process.CloseMainWindow() | Out-Null }
}
