param(
    [string]$BuildDirectory = (Join-Path $PSScriptRoot '..\build_portable'),
    [string]$OutputDirectory = (Join-Path $PSScriptRoot '..\dist')
)

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$buildRoot = [System.IO.Path]::GetFullPath($BuildDirectory)
$outputRoot = [System.IO.Path]::GetFullPath($OutputDirectory)
$productBaseName = 'AeroSys_Lab_v4.1_Windows_x64'
$sourceExe = Join-Path $buildRoot 'Release\AeroSysLab.exe'
$singleExe = Join-Path $outputRoot ($productBaseName + '.exe')
$zipPath = Join-Path $outputRoot ($productBaseName + '_Portable.zip')
$stagingDirectory = Join-Path $outputRoot ($productBaseName + '_Portable')

cmake -S $projectRoot -B $buildRoot -A x64
if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
cmake --build $buildRoot --config Release --parallel
if ($LASTEXITCODE -ne 0) { throw 'Release build failed.' }
ctest --test-dir $buildRoot -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Automated tests failed.' }

$resourceCheck = Start-Process -FilePath $sourceExe -ArgumentList '--verify-embedded-data' `
    -WindowStyle Hidden -Wait -PassThru
if ($resourceCheck.ExitCode -ne 0) {
    throw "Embedded mission-data validation failed with exit code $($resourceCheck.ExitCode)."
}

New-Item -ItemType Directory -Force -Path $outputRoot | Out-Null
Copy-Item -LiteralPath $sourceExe -Destination $singleExe -Force

$expectedPrefix = $outputRoot.TrimEnd('\') + '\'
$resolvedStaging = [System.IO.Path]::GetFullPath($stagingDirectory)
if (-not $resolvedStaging.StartsWith($expectedPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'Portable staging directory resolved outside the output directory.'
}
if (Test-Path -LiteralPath $resolvedStaging) {
    Remove-Item -LiteralPath $resolvedStaging -Recurse -Force
}
New-Item -ItemType Directory -Path $resolvedStaging | Out-Null
Copy-Item -LiteralPath $singleExe -Destination (Join-Path $resolvedStaging ($productBaseName + '.exe'))
Copy-Item -LiteralPath (Join-Path $projectRoot 'packaging\README_portable.txt') `
    -Destination (Join-Path $resolvedStaging 'README.txt')

$hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $singleExe).Hash
Set-Content -Encoding ASCII -LiteralPath (Join-Path $resolvedStaging 'SHA256.txt') `
    -Value ($hash + '  ' + $productBaseName + '.exe')
foreach ($requiredFile in @(($productBaseName + '.exe'), 'README.txt', 'SHA256.txt')) {
    if (-not (Test-Path -LiteralPath (Join-Path $resolvedStaging $requiredFile))) {
        throw "Portable staging file is missing: $requiredFile"
    }
}
if (Test-Path -LiteralPath $zipPath) { Remove-Item -LiteralPath $zipPath -Force }
Compress-Archive -Path (Join-Path $resolvedStaging '*') -DestinationPath $zipPath -CompressionLevel Optimal
Remove-Item -LiteralPath $resolvedStaging -Recurse -Force

Write-Output "Single EXE: $singleExe"
Write-Output "Portable ZIP: $zipPath"
Write-Output "SHA-256: $hash"
