# Public HTTP workflow for a server already listening on loopback.
# Place Basic.ini in the configured profiles/vendor directory before startup.
# Supply REST_SMOKE_PASSWORD in the process environment (never an argv password).
param([string]$BaseUrl = 'http://127.0.0.1:8080', [string]$Username = 'Smoke', [string]$OutputDirectory = '.')
$ErrorActionPreference = 'Stop'
if (-not $env:REST_SMOKE_PASSWORD) { throw 'Set REST_SMOKE_PASSWORD for this smoke run' }
$headers = @{}
function Request([string]$Method, [string]$Path, $Body = $null, [string]$ContentType = 'application/json') {
    $parameters = @{ Uri = "$BaseUrl$Path"; Method = $Method; Headers = $headers; UseBasicParsing = $true; TimeoutSec = 300 }
    if ($null -ne $Body) {
        $parameters.Body = [Text.Encoding]::UTF8.GetBytes([string]$Body)
        $parameters.ContentType = $ContentType
    }
    Invoke-WebRequest @parameters
}
function JsonRequest([string]$Method, [string]$Path, $Body = $null) {
    $response = Request $Method $Path $Body
    if ($response.Content) { $response.Content | ConvertFrom-Json }
}
$token = (JsonRequest POST '/login' (@{username=$Username; password=$env:REST_SMOKE_PASSWORD} | ConvertTo-Json)).access_token
if (-not $token) { throw 'Login returned no token' }
$headers.Authorization = "Bearer $token"
$id = (JsonRequest POST '/create_session').session_id
$route = '/session/' + [Uri]::EscapeDataString($id)
$profiles = JsonRequest GET "$route/list_profiles"
function ProfileId($items, [string]$Name) {
    $item = @($items | Where-Object { $_.name -eq $Name })
    if ($item.Count -ne 1) { throw "Expected one Basic profile: $Name" }
    $item[0].id
}
$selection = @{print=(ProfileId $profiles.print '0.20mm NORMAL .4N');
    printer=(ProfileId $profiles.printer 'Basic 0.4 nozzle'); extruder=@();
    filaments=@((ProfileId $profiles.filaments 'Basic PLA @BASIC'))} | ConvertTo-Json -Depth 5
$null = JsonRequest POST "$route/set_profile" $selection
# Generate a closed ASCII STL cube without requiring an external model file.
$vertices = @('0 0 0', '10 0 0', '10 10 0', '0 10 0', '0 0 10', '10 0 10', '10 10 10', '0 10 10')
$facets = @(@(1,3,2), @(1,4,3), @(5,6,7), @(5,7,8), @(1,2,6), @(1,6,5),
    @(2,3,7), @(2,7,6), @(3,4,8), @(3,8,7), @(4,1,5), @(4,5,8))
$stl = [Text.StringBuilder]::new("solid smoke`n")
foreach ($facet in $facets) {
    $null = $stl.Append("facet normal 0 0 0`nouter loop`n")
    foreach ($index in $facet) { $null = $stl.Append("vertex $($vertices[$index - 1])`n") }
    $null = $stl.Append("endloop`nendfacet`n")
}
$null = $stl.Append("endsolid smoke`n")
$boundary = 'restSmokeBoundary'
$multipart = "--$boundary`r`nContent-Disposition: form-data; name=`"file`"; filename=`"cube.stl`"`r`nContent-Type: application/octet-stream`r`n`r`n$stl`r`n--$boundary--`r`n"
$null = Request POST "$route/add_model" $multipart "multipart/form-data; boundary=$boundary"
$null = Request POST "$route/slice"
$deadline = [DateTime]::UtcNow.AddSeconds(300)
do {
    $state = JsonRequest GET "$route/state"
    if ($state.etat -eq 'sliced') { break }
    if ($state.etat -ne 'slicing') { throw "Slice failed: $($state.etat)" }
    if ([DateTime]::UtcNow -gt $deadline) {
        throw ('Slice did not finish: ' + ($state | ConvertTo-Json -Compress))
    }
    Start-Sleep -Milliseconds 200
} while ($true)
$null = New-Item -ItemType Directory -Force -Path $OutputDirectory
$output = Join-Path ([IO.Path]::GetFullPath($OutputDirectory)) 'smoke.gcode'
# OutFile preserves the HTTP bytes regardless of PowerShell's Content decoding.
Invoke-WebRequest -Uri "$BaseUrl$route/get_gcode" -Headers $headers -UseBasicParsing -TimeoutSec 300 -OutFile $output
if ([IO.File]::ReadAllText($output) -notmatch 'G1') { throw 'Downloaded G-code is empty or invalid' }
$null = Request DELETE "$route/delete"
$null = Request POST '/logout'
Write-Output 'PASS login, Basic profiles, model upload, slice, download, delete, logout'
