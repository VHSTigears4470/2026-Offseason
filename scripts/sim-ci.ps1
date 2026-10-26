# Publishes box3d-sim locally, then runs headless sim cases and fails if any case fails.
param(
    [string]$LibraryPath = 'F:\Projects\box3d-sim',
    [int]$AutoMinScore = 6
)
$ErrorActionPreference = 'Stop'
Set-Location (Join-Path $PSScriptRoot '..')

if (-not $env:JAVA_HOME) { $env:JAVA_HOME = 'C:\Users\Public\wpilib\2027_alpha7\jdk' }
$version = & "$env:JAVA_HOME\bin\java.exe" -version 2>&1 | Select-Object -First 1
if ($version -notmatch '"25') { throw "JAVA_HOME must be a JDK 25 (found: $version)" }

& (Join-Path $LibraryPath 'gradlew.bat') -p $LibraryPath publishToMavenLocal --console=plain -q
if ($LASTEXITCODE -ne 0) { throw "box3d-sim publishToMavenLocal failed ($LASTEXITCODE)" }

$cases = @(
    @{ Name = 'auto-smoke'; Args = @('-PsimMode=fast', '-PsimAuto=Shoot at Hub', '-PsimMatch=auto', "-PsimExpectMinScore=$AutoMinScore") },
    @{ Name = 'full-match'; Args = @('-PsimMode=fast', '-PsimMatch=full') }
)
New-Item -ItemType Directory -Force build/sim-ci | Out-Null
$results = foreach ($case in $cases) {
    Remove-Item build/sim-results.json -ErrorAction SilentlyContinue
    $output = & .\gradlew.bat run @($case.Args) --console=plain 2>&1
    $code = $LASTEXITCODE
    $line = $output | Where-Object { $_ -match '^SIM RESULT:' } | Select-Object -Last 1
    if (Test-Path build/sim-results.json) { Copy-Item build/sim-results.json "build/sim-ci/$($case.Name).json" -Force }
    [pscustomobject]@{ Case = $case.Name; Exit = $code; Result = $line }
}
$results | Format-Table -AutoSize
if ($results | Where-Object { $_.Exit -ne 0 }) { exit 1 }
