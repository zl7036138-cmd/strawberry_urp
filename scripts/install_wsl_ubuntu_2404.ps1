[CmdletBinding()]
param(
    [string]$Distribution = 'Ubuntu-24.04-URP',
    [string]$ImagePath = '',
    [string]$InstallLocation = "$env:USERPROFILE\WSL\Ubuntu-24.04-URP"
)

$ErrorActionPreference = 'Stop'

$installed = (wsl.exe --list --quiet 2>$null) -replace "`0", ''
if ($installed -contains $Distribution) {
    Write-Host "$Distribution is already installed."
    exit 0
}

Write-Host "Installing $Distribution without changing existing WSL distributions..."
if ($ImagePath) {
    $resolvedImage = (Resolve-Path -LiteralPath $ImagePath -ErrorAction Stop).Path
    if (-not [System.IO.File]::Exists($resolvedImage)) {
        throw "WSL image is not a file: $resolvedImage"
    }
    $resolvedLocation = [System.IO.Path]::GetFullPath($InstallLocation)
    New-Item -ItemType Directory -Force -Path $resolvedLocation | Out-Null
    wsl.exe --install --from-file $resolvedImage --name $Distribution `
        --location $resolvedLocation --version 2 --no-launch
} else {
    wsl.exe --install -d Ubuntu-24.04 --web-download --name $Distribution `
        --location $InstallLocation --version 2 --no-launch
}
if ($LASTEXITCODE -ne 0) {
    throw "WSL installation failed with exit code $LASTEXITCODE"
}

Write-Host "$Distribution was installed. Bootstrap it as root with:"
Write-Host "  wsl -d $Distribution -u root -- bash /mnt/c/.../scripts/bootstrap_ubuntu_2404.sh"
