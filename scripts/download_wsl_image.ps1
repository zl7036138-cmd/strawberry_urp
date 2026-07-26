[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Uri,
    [Parameter(Mandatory = $true)][string]$OutputPath,
    [Parameter(Mandatory = $true)][long]$ExpectedSize,
    [Parameter(Mandatory = $true)][string]$ExpectedSha256,
    [ValidateRange(1, 16)][int]$Parts = 8
)

$ErrorActionPreference = 'Stop'
$OutputPath = [System.IO.Path]::GetFullPath($OutputPath)
$outputDirectory = Split-Path -Parent $OutputPath
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$partDirectory = "$OutputPath.parts"
New-Item -ItemType Directory -Force -Path $partDirectory | Out-Null
$curlPath = (Get-Command curl.exe -ErrorAction Stop).Source

$chunkSize = [long][math]::Ceiling($ExpectedSize / [double]$Parts)
$jobs = @()
for ($index = 0; $index -lt $Parts; $index++) {
    $start = [long]$index * $chunkSize
    $end = [math]::Min($ExpectedSize - 1, $start + $chunkSize - 1)
    $partPath = Join-Path $partDirectory ("part-{0:D2}.bin" -f $index)
    $expectedPartSize = $end - $start + 1
    if (Test-Path -LiteralPath $partPath) {
        $existing = (Get-Item -LiteralPath $partPath).Length
        if ($existing -gt $expectedPartSize) {
            Remove-Item -LiteralPath $partPath -Force
        }
    }

    $jobs += Start-Job -ArgumentList @(
        $curlPath, $Uri, $partPath, $start, $end, $expectedPartSize, $index
    ) -ScriptBlock {
        param($CurlPath, $Uri, $PartPath, $Start, $End, $ExpectedPartSize, $Index)
        $tempPath = "$PartPath.download"
        Remove-Item -LiteralPath $tempPath -Force -ErrorAction SilentlyContinue
        $attempt = 0
        while ($true) {
            $current = if (Test-Path -LiteralPath $PartPath) {
                (Get-Item -LiteralPath $PartPath).Length
            } else {
                0L
            }
            if ($current -eq $ExpectedPartSize) {
                break
            }
            if ($current -gt $ExpectedPartSize) {
                throw "part $Index exceeds its expected length"
            }
            $attempt++
            if ($attempt -gt 40) {
                throw "part $Index exceeded the reconnect limit"
            }
            $rangeStart = [long]$Start + $current
            # curl writes transport fragments to disk even when the connection
            # ends early.  The next iteration resumes from the exact byte count.
            & $CurlPath `
                --location `
                --fail `
                --silent `
                --connect-timeout 20 `
                --max-time 180 `
                --range "$rangeStart-$End" `
                --output $tempPath `
                $Uri `
                2>$null
            $curlExitCode = $LASTEXITCODE
            if (Test-Path -LiteralPath $tempPath) {
                $downloaded = (Get-Item -LiteralPath $tempPath).Length
                $remaining = $ExpectedPartSize - $current
                if ($downloaded -gt $remaining) {
                    throw "part $Index server returned $downloaded bytes, only $remaining remain"
                }
                if ($downloaded -gt 0) {
                    $input = [System.IO.File]::OpenRead($tempPath)
                    $file = [System.IO.File]::Open(
                        $PartPath,
                        [System.IO.FileMode]::Append,
                        [System.IO.FileAccess]::Write,
                        [System.IO.FileShare]::Read
                    )
                    try { $input.CopyTo($file) } finally {
                        $file.Dispose()
                        $input.Dispose()
                    }
                }
                Remove-Item -LiteralPath $tempPath -Force
                if ($downloaded -eq 0) {
                    Start-Sleep -Seconds ([math]::Min($attempt, 5))
                }
            } else {
                Start-Sleep -Seconds ([math]::Min($attempt, 5))
            }
            if (($curlExitCode -ne 0) -and ($attempt -eq 40)) {
                throw "part $Index curl failed with exit code $curlExitCode"
            }
        }
        [PSCustomObject]@{ Index = $Index; Bytes = $ExpectedPartSize }
    }
}

$jobs | Wait-Job | Out-Null
$jobOutput = $jobs | Receive-Job
$failed = $jobs | Where-Object State -ne 'Completed'
if ($failed) {
    $jobs | Format-List Id, State, HasMoreData
    throw "one or more range download jobs failed"
}
$jobs | Remove-Job
$jobOutput | Sort-Object Index | Format-Table -AutoSize

$output = [System.IO.File]::Open(
    $OutputPath,
    [System.IO.FileMode]::Create,
    [System.IO.FileAccess]::Write,
    [System.IO.FileShare]::None
)
try {
    for ($index = 0; $index -lt $Parts; $index++) {
        $partPath = Join-Path $partDirectory ("part-{0:D2}.bin" -f $index)
        $input = [System.IO.File]::OpenRead($partPath)
        try { $input.CopyTo($output) } finally { $input.Dispose() }
    }
} finally {
    $output.Dispose()
}

$actualSize = (Get-Item -LiteralPath $OutputPath).Length
if ($actualSize -ne $ExpectedSize) {
    throw "assembled file has $actualSize bytes, expected $ExpectedSize"
}
$actualHash = (Get-FileHash -LiteralPath $OutputPath -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actualHash -ne $ExpectedSha256.ToLowerInvariant()) {
    throw "SHA256 mismatch: expected $ExpectedSha256, got $actualHash"
}

$resolvedParent = (Resolve-Path -LiteralPath $outputDirectory).Path.TrimEnd('\')
$resolvedParts = (Resolve-Path -LiteralPath $partDirectory).Path
if (-not $resolvedParts.StartsWith($resolvedParent + '\', [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "refusing to remove range parts outside output directory: $resolvedParts"
}
Remove-Item -LiteralPath $resolvedParts -Recurse -Force
Write-Host "Verified download: $OutputPath ($actualSize bytes, sha256=$actualHash)"
