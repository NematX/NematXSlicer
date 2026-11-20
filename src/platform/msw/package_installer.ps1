# Assemble a WiX installer from a staged portable package. The caller decides
# whether to sign that package first. Keep WiX build files and the output MSI
# out of the portable directory, and remove the temporary license after linking.
param(
    [Parameter(Mandatory = $true)][string]$Package,
    [Parameter(Mandatory = $true)][string]$WixSource,
    [Parameter(Mandatory = $true)][string]$InstallerVersion,
    [Parameter(Mandatory = $true)][string]$OutputPath,
    [string]$ReleaseSuffix = ''
)

$ErrorActionPreference = 'Stop'
if ($InstallerVersion -cnotmatch '\A[0-9]+(?:\.[0-9]+){2,3}\z') {
    throw 'The installer version must have three or four numeric components.'
}
if (!$env:WIX) { throw 'WiX 3 was not found on the Windows runner.' }
$bin = Join-Path $env:WIX 'bin'
foreach ($name in @('heat.exe', 'candle.exe', 'light.exe')) {
    if (!(Test-Path (Join-Path $bin $name) -PathType Leaf)) { throw "Missing WiX tool: $name" }
}

$Package = (Resolve-Path -LiteralPath $Package).Path
$WixSource = (Resolve-Path -LiteralPath $WixSource).Path
$OutputPath = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutputPath)
$licenseRtf = Join-Path $Package 'LICENSE.rtf'
if (Test-Path -LiteralPath $licenseRtf) { throw "Unexpected portable file: $licenseRtf" }

Push-Location (Split-Path -Parent $WixSource)
try {
    $license = Get-Content -LiteralPath (Join-Path $Package 'LICENSE') -Raw
    $rtf = "{\rtf1\ansi\deff0 {\fonttbl {\f0 Times New Roman;}}{\f0\fs20 " + $license.Replace('\', '\\').Replace("`n", "\par ") + "}}"
    $rtf | Out-File -LiteralPath $licenseRtf -Encoding ASCII

    & (Join-Path $bin 'heat.exe') dir $Package -dr INSTALLFOLDER -cg Package -gg -sreg -srd -var var.SourceDir -out files.wxs -sfrag
    if ($LASTEXITCODE -ne 0) { throw "WiX heat failed with exit code $LASTEXITCODE." }
    & (Join-Path $bin 'candle.exe') "-dSourceDir=$Package" "-dInstallerVersion=$InstallerVersion" "-dReleaseSuffix=$ReleaseSuffix" $WixSource files.wxs
    if ($LASTEXITCODE -ne 0) { throw "WiX candle failed with exit code $LASTEXITCODE." }
    & (Join-Path $bin 'light.exe') -b $Package -ext WixUIExtension winInstaller.wixobj files.wixobj -o $OutputPath -spdb -sw1076
    if ($LASTEXITCODE -ne 0) { throw "WiX light failed with exit code $LASTEXITCODE." }
} finally {
    if (Test-Path -LiteralPath $licenseRtf) { Remove-Item -LiteralPath $licenseRtf }
    Pop-Location
}
