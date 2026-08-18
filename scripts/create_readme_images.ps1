param(
    [string]$ArtifactDirectory = (Join-Path $PSScriptRoot '..\artifacts'),
    [string]$OutputDirectory = (Join-Path $PSScriptRoot '..\docs\images')
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing

New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
Copy-Item -LiteralPath (Join-Path $ArtifactDirectory 'satellite_running.png') `
    -Destination (Join-Path $OutputDirectory 'aerognc-main-overview.png') -Force

$canvas = New-Object System.Drawing.Bitmap 1700, 1400
$graphics = [System.Drawing.Graphics]::FromImage($canvas)
$graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::HighQuality
$graphics.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
$graphics.PixelOffsetMode = [System.Drawing.Drawing2D.PixelOffsetMode]::HighQuality
$background = [System.Drawing.Color]::FromArgb(5, 11, 18)
$panel = [System.Drawing.Color]::FromArgb(11, 22, 33)
$border = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(32, 55, 71)), 2
$textBrush = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(229, 238, 243))
$titleFont = New-Object System.Drawing.Font 'Microsoft YaHei UI', 25, ([System.Drawing.FontStyle]::Bold)
$labelFont = New-Object System.Drawing.Font 'Microsoft YaHei UI', 16, ([System.Drawing.FontStyle]::Bold)
$graphics.Clear($background)
$graphics.DrawString('AeroGNC Lab v3.2 | Control & Simulation Settings',
    $titleFont, $textBrush, 40, 24)

function Draw-PanelImage {
    param(
        [System.Drawing.Graphics]$Graphics,
        [string]$Path,
        [string]$Label,
        [int]$X,
        [int]$Y
    )
    $panelRect = New-Object System.Drawing.Rectangle $X, $Y, 800, 625
    $panelBrush = New-Object System.Drawing.SolidBrush $panel
    $Graphics.FillRectangle($panelBrush, $panelRect)
    $Graphics.DrawRectangle($border, $panelRect)
    $Graphics.DrawString($Label, $labelFont, $textBrush, ($X + 18), ($Y + 12))
    $source = [System.Drawing.Image]::FromFile($Path)
    try {
        $availableWidth = 764
        $availableHeight = 552
        $scale = [Math]::Min($availableWidth / $source.Width, $availableHeight / $source.Height)
        $width = [int]($source.Width * $scale)
        $height = [int]($source.Height * $scale)
        $left = $X + [int]((800 - $width) / 2)
        $top = $Y + 60 + [int](($availableHeight - $height) / 2)
        $Graphics.DrawImage($source, $left, $top, $width, $height)
    }
    finally {
        $source.Dispose()
        $panelBrush.Dispose()
    }
}

Draw-PanelImage $graphics (Join-Path $ArtifactDirectory 'control_settings.png') `
    'Attitude Control' 35 90
Draw-PanelImage $graphics (Join-Path $ArtifactDirectory 'satellite_orbit_control_settings.png') `
    'Satellite Orbit Control' 865 90
Draw-PanelImage $graphics (Join-Path $ArtifactDirectory 'rocket_guidance_settings.png') `
    'Rocket Trajectory Guidance' 35 745
Draw-PanelImage $graphics (Join-Path $ArtifactDirectory 'satellite_disturbance_settings.png') `
    'Disturbance Settings' 865 745

$outputPath = Join-Path $OutputDirectory 'aerognc-settings-overview.png'
$canvas.Save($outputPath, [System.Drawing.Imaging.ImageFormat]::Png)
$labelFont.Dispose()
$titleFont.Dispose()
$textBrush.Dispose()
$border.Dispose()
$graphics.Dispose()
$canvas.Dispose()

Write-Output "README images created in $OutputDirectory"
