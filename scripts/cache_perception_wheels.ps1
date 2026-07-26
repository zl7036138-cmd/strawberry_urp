[CmdletBinding()]
param(
    [string]$LockFile = '',
    [string]$OutputDirectory = ''
)

$ErrorActionPreference = 'Stop'
$repoRoot = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..')
)
if (-not $LockFile) {
    $LockFile = Join-Path $repoRoot `
        'requirements\perception-linux-cp312-wheelhouse.txt'
}
if (-not $OutputDirectory) {
    $OutputDirectory = Join-Path $repoRoot '.cache\wheels'
}
$lockPath = (Resolve-Path -LiteralPath $LockFile -ErrorAction Stop).Path
$outputPath = [System.IO.Path]::GetFullPath($OutputDirectory)
if (-not $outputPath.StartsWith(
    $repoRoot.TrimEnd('\') + '\',
    [System.StringComparison]::OrdinalIgnoreCase
)) {
    throw "wheel output must stay inside the repository: $outputPath"
}
New-Item -ItemType Directory -Force -Path $outputPath | Out-Null
Add-Type -AssemblyName System.Net.Http
$httpHandler = New-Object System.Net.Http.HttpClientHandler
$httpClient = New-Object System.Net.Http.HttpClient($httpHandler)
$httpClient.Timeout = [TimeSpan]::FromMinutes(2)

$requirements = @(
    Get-Content -LiteralPath $lockPath -Encoding UTF8 |
        ForEach-Object { $_.Trim() } |
        Where-Object { $_ -and -not $_.StartsWith('#') }
)
if ($requirements.Count -lt 30) {
    throw "expected at least 30 pinned wheelhouse requirements"
}

function Get-WheelScore {
    param([string]$FileName)
    $lower = $FileName.ToLowerInvariant()
    if ($lower -notlike '*.whl') { return -1 }
    if ($lower -match '(aarch64|arm64|ppc64|s390x|win32|win_amd64|macosx)') {
        return -1
    }
    if ($lower -match '-cp312-cp312-.*x86_64\.whl$') { return 100 }
    if ($lower -match '-cp3\d+-abi3-.*x86_64\.whl$') { return 90 }
    if ($lower -match '-py3-none-.*x86_64\.whl$') { return 80 }
    if ($lower -match '-py3-none-any\.whl$') { return 70 }
    if ($lower -match '-py2\.py3-none-any\.whl$') { return 70 }
    return -1
}

$receipt = @()
$index = 0
foreach ($requirement in $requirements) {
    $index++
    if ($requirement -notmatch '^([A-Za-z0-9_.-]+)==([A-Za-z0-9_.+-]+)$') {
        throw "invalid pinned requirement: $requirement"
    }
    $name = $Matches[1]
    $version = $Matches[2]
    Write-Host "[$index/$($requirements.Count)] resolving $name==$version"
    $metadataUri = "https://pypi.org/pypi/$name/$version/json"
    $metadata = Invoke-RestMethod -Uri $metadataUri -TimeoutSec 60
    $candidates = @(
        $metadata.urls |
            Where-Object { $_.packagetype -eq 'bdist_wheel' } |
            ForEach-Object {
                [PSCustomObject]@{
                    Score = Get-WheelScore -FileName $_.filename
                    File = $_
                }
            } |
            Where-Object Score -ge 0 |
            Sort-Object -Property `
                @{Expression = 'Score'; Descending = $true}, `
                @{Expression = { $_.File.filename }; Ascending = $true}
    )
    if (-not $candidates) {
        throw "no CPython 3.12 Linux x86_64-compatible wheel for $requirement"
    }
    $selected = $candidates[0].File
    $destination = Join-Path $outputPath $selected.filename
    $expectedHash = $selected.digests.sha256.ToLowerInvariant()
    $downloadRequired = $true
    if (Test-Path -LiteralPath $destination) {
        $existingHash = (
            Get-FileHash -LiteralPath $destination -Algorithm SHA256
        ).Hash.ToLowerInvariant()
        if ($existingHash -eq $expectedHash) {
            $downloadRequired = $false
            Write-Host "  verified cached $($selected.filename)"
        } else {
            Remove-Item -LiteralPath $destination -Force
        }
    }
    if ($downloadRequired) {
        $temporary = "$destination.download"
        $expectedSize = [long]$selected.size
        if ((Test-Path -LiteralPath $temporary) -and
            (Get-Item -LiteralPath $temporary).Length -gt $expectedSize) {
            Remove-Item -LiteralPath $temporary -Force
        }
        $attempt = 0
        while (
            (-not (Test-Path -LiteralPath $temporary)) -or
            (Get-Item -LiteralPath $temporary).Length -lt $expectedSize
        ) {
            $attempt++
            if ($attempt -gt 12) {
                throw "HTTP download retry limit exceeded for $requirement"
            }
            $existingSize = if (Test-Path -LiteralPath $temporary) {
                (Get-Item -LiteralPath $temporary).Length
            } else {
                0L
            }
            $request = New-Object `
                -TypeName System.Net.Http.HttpRequestMessage `
                -ArgumentList @(
                    [System.Net.Http.HttpMethod]::Get,
                    [Uri]$selected.url
                )
            if ($existingSize -gt 0) {
                $request.Headers.Range = (
                    [System.Net.Http.Headers.RangeHeaderValue]::Parse(
                        "bytes=$existingSize-"
                    )
                )
            }
            $response = $null
            $source = $null
            $target = $null
            try {
                $response = $httpClient.SendAsync(
                    $request,
                    [System.Net.Http.HttpCompletionOption]::ResponseHeadersRead
                ).GetAwaiter().GetResult()
                $response.EnsureSuccessStatusCode() | Out-Null
                if (
                    $existingSize -gt 0 -and
                    $response.StatusCode -ne
                        [System.Net.HttpStatusCode]::PartialContent
                ) {
                    Remove-Item -LiteralPath $temporary -Force
                    continue
                }
                $streamTask = $response.Content.ReadAsStreamAsync()
                $source = $streamTask.GetAwaiter().GetResult()
                $fileMode = if ($existingSize -gt 0) {
                    [System.IO.FileMode]::Append
                } else {
                    [System.IO.FileMode]::Create
                }
                $target = New-Object `
                    -TypeName System.IO.FileStream `
                    -ArgumentList @(
                        $temporary,
                        $fileMode,
                        [System.IO.FileAccess]::Write,
                        [System.IO.FileShare]::Read
                )
                $copyTask = $source.CopyToAsync($target)
                $null = $copyTask.GetAwaiter().GetResult()
            } catch {
                Write-Warning (
                    "download attempt $attempt failed for $requirement`: " +
                    $_.Exception.Message
                )
                Start-Sleep -Seconds ([math]::Min($attempt * 2, 15))
            } finally {
                if ($target) { $target.Dispose() }
                if ($source) { $source.Dispose() }
                if ($response) { $response.Dispose() }
                $request.Dispose()
            }
        }
        if ((Get-Item -LiteralPath $temporary).Length -ne $expectedSize) {
            throw "size mismatch for $requirement"
        }
        $actualHash = (
            Get-FileHash -LiteralPath $temporary -Algorithm SHA256
        ).Hash.ToLowerInvariant()
        if ($actualHash -ne $expectedHash) {
            throw "SHA256 mismatch for $requirement"
        }
        Move-Item -LiteralPath $temporary -Destination $destination
        Write-Host "  downloaded $($selected.filename)"
    }
    $file = Get-Item -LiteralPath $destination
    $receipt += [PSCustomObject]@{
        requirement = $requirement
        filename = $selected.filename
        size_bytes = $file.Length
        sha256 = $expectedHash
        url = $selected.url
    }
}

$receiptPath = Join-Path $outputPath 'wheelhouse-manifest.json'
$receiptPayload = [ordered]@{
    schema_version = 1
    kind = 'perception_linux_cp312_wheelhouse'
    lock_file = $lockPath.Substring($repoRoot.Length + 1).Replace('\', '/')
    package_count = $receipt.Count
    packages = $receipt
}
$receiptPayload |
    ConvertTo-Json -Depth 5 |
    Set-Content -LiteralPath $receiptPath -Encoding UTF8
Write-Host "Wheelhouse complete: $($receipt.Count) packages"
Write-Host "Manifest: $receiptPath"
$httpClient.Dispose()
$httpHandler.Dispose()
