param(
    [string]$LogPath = "artifacts/autofix-repro.log"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

Write-Host "Running deterministic CI autofix hook..."

function Add-LineAfterAnchor {
    param(
        [Parameter(Mandatory = $true)][string]$FilePath,
        [Parameter(Mandatory = $true)][string]$AnchorLine,
        [Parameter(Mandatory = $true)][string]$LineToInsert
    )

    if (-not (Test-Path $FilePath)) {
        Write-Host "Autofix: file not found, skipping: $FilePath"
        return $false
    }

    $raw = Get-Content -Path $FilePath -Raw
    $escapedInsert = [regex]::Escape($LineToInsert)
    if ($raw -match "(?m)^$escapedInsert\r?$") {
        return $false
    }

    $escapedAnchor = [regex]::Escape($AnchorLine)
    if ($raw -notmatch "(?m)^$escapedAnchor\r?$") {
        Write-Host "Autofix: anchor not found in $FilePath: $AnchorLine"
        return $false
    }

    $updated = [regex]::Replace(
        $raw,
        "(?m)^$escapedAnchor\r?$",
        "$AnchorLine`n$LineToInsert",
        1)

    if ($updated -ne $raw) {
        [System.IO.File]::WriteAllText($FilePath, $updated, [System.Text.UTF8Encoding]::new($false))
        Write-Host "Autofix: inserted '$LineToInsert' into $FilePath"
        return $true
    }

    return $false
}

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
# Rule 2: repair missing type aliases in 3D capability tests.
# This addresses known CI failures with MSVC errors such as:
# - C4430/C2146 around BrepLoop / Point3d / Vector3d
# - C3861 'Cross': identifier not found
if (Test-Path $LogPath) {
    $logText = Get-Content -Path $LogPath -Raw

    $sawBrepAliasFailure = $logText -match "test_3d_brep\.cpp" -and (
        $logText -match "missing ';' before identifier 'originalLoop'" -or
        $logText -match "redefinition.+BrepLoop")

    if ($sawBrepAliasFailure) {
        [void](Add-LineAfterAnchor `
            -FilePath "tests/capabilities/test_3d_brep.cpp" `
            -AnchorLine "using geometry::sdk::BrepBodyEdit3d;" `
            -LineToInsert "using geometry::sdk::BrepLoop;")
    }

    $sawConversionAliasFailure = $logText -match "test_3d_conversion\.cpp" -and (
        $logText -match "missing ';' before identifier 'SkewPoint'" -or
        $logText -match "'Point3d': redefinition" -or
        $logText -match "'Vector3d': undeclared identifier" -or
        $logText -match "'Cross': identifier not found" -or
        $logText -match "'Point2d': undeclared identifier")

    if ($sawConversionAliasFailure) {
        [void](Add-LineAfterAnchor `
            -FilePath "tests/capabilities/test_3d_conversion.cpp" `
            -AnchorLine "using geometry::sdk::PlaneSurface;" `
            -LineToInsert "using geometry::sdk::Point2d;")

        [void](Add-LineAfterAnchor `
            -FilePath "tests/capabilities/test_3d_conversion.cpp" `
            -AnchorLine "using geometry::sdk::PlaneSurface;" `
            -LineToInsert "using geometry::sdk::Point3d;")

        [void](Add-LineAfterAnchor `
            -FilePath "tests/capabilities/test_3d_conversion.cpp" `
            -AnchorLine "using geometry::sdk::Surface;" `
            -LineToInsert "using geometry::sdk::Vector3d;")

        [void](Add-LineAfterAnchor `
            -FilePath "tests/capabilities/test_3d_conversion.cpp" `
            -AnchorLine "using geometry::sdk::Vector3d;" `
            -LineToInsert "using geometry::Cross;")
    }
} else {
    Write-Host "Autofix: log file not found at '$LogPath', skipping log-driven rules."
}

Write-Host "Deterministic CI autofix hook finished."