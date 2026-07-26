[CmdletBinding()]
param(
    [string]$RepositoryRoot = '',
    [string]$RosdistroCommit = 'f8070f9f7cf974eec72491b0a5622c6824d46a3f'
)

$ErrorActionPreference = 'Stop'

if (-not $RepositoryRoot) {
    $RepositoryRoot = Split-Path -Parent $PSScriptRoot
}
$RepositoryRoot = (Resolve-Path -LiteralPath $RepositoryRoot -ErrorAction Stop).Path
$cacheDirectory = Join-Path $RepositoryRoot '.cache\rosdep'
[System.IO.Directory]::CreateDirectory($cacheDirectory) | Out-Null

$baseUri = "https://raw.githubusercontent.com/ros/rosdistro/$RosdistroCommit"
$sources = [ordered]@{
    'base.yaml' = "$baseUri/rosdep/base.yaml"
    'python.yaml' = "$baseUri/rosdep/python.yaml"
    'ruby.yaml' = "$baseUri/rosdep/ruby.yaml"
    'index-v4.yaml' = "$baseUri/index-v4.yaml"
    'jazzy-distribution.yaml' = "$baseUri/jazzy/distribution.yaml"
}
$curlPath = (Get-Command curl.exe -ErrorAction Stop).Source

foreach ($entry in $sources.GetEnumerator()) {
    $destination = Join-Path $cacheDirectory $entry.Key
    $temporary = "$destination.download"
    [System.IO.File]::Delete($temporary)

    & $curlPath `
        --fail `
        --location `
        --retry 10 `
        --retry-all-errors `
        --retry-delay 2 `
        --connect-timeout 20 `
        --max-time 300 `
        --output $temporary `
        $entry.Value
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to download $($entry.Value) (curl exit $LASTEXITCODE)"
    }
    if (-not (Test-Path -LiteralPath $temporary -PathType Leaf) -or
        (Get-Item -LiteralPath $temporary).Length -eq 0) {
        throw "Downloaded rosdep source is empty: $($entry.Key)"
    }
    [System.IO.File]::Delete($destination)
    [System.IO.File]::Move($temporary, $destination)
}

$indexText = Get-Content -LiteralPath (Join-Path $cacheDirectory 'index-v4.yaml') -Raw
$distributionText = Get-Content -LiteralPath (
    Join-Path $cacheDirectory 'jazzy-distribution.yaml'
) -Raw
if ($indexText -notmatch '(?m)^  jazzy:$' -or
    $distributionText -notmatch '(?m)^repositories:$') {
    throw 'Cached rosdistro files failed structural validation.'
}

$manifest = Get-ChildItem -LiteralPath $cacheDirectory -Filter '*.yaml' |
    Sort-Object Name |
    ForEach-Object {
        [PSCustomObject]@{
            file = $_.Name
            sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
            bytes = $_.Length
        }
    }
$manifest | ConvertTo-Json | Set-Content -LiteralPath (
    Join-Path $cacheDirectory 'manifest.json'
) -Encoding utf8

Write-Host "Cached pinned rosdep sources in $cacheDirectory"
Write-Host "rosdistro commit: $RosdistroCommit"
