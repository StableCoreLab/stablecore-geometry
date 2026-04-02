param(
    [string]$LogPath = "artifacts/autofix-repro.log"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

Write-Host "Running deterministic CI autofix hook..."

# Rule 1 (safe default): normalize CRLF/LF for workflow and markdown files.
# This avoids noisy line-ending diffs that can occasionally break strict checks.
$targets = @(
    ".github/workflows/*.yml",
    "docs/*.md"
)

foreach ($pattern in $targets) {
    Get-ChildItem -Path $pattern -File -ErrorAction SilentlyContinue | ForEach-Object {
        $raw = Get-Content -Path $_.FullName -Raw
        # Normalize to LF in repo content.
        $normalized = $raw -replace "`r`n", "`n"
        if ($normalized -ne $raw) {
            [System.IO.File]::WriteAllText($_.FullName, $normalized, [System.Text.UTF8Encoding]::new($false))
            Write-Host "Normalized line endings: $($_.FullName)"
        }
    }
}

# Rule 2 placeholder: add future deterministic text/code transforms here.
Write-Host "Deterministic CI autofix hook finished."