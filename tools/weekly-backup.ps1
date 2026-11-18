# Weekly auto-save / sync of the CG_Scale project with GitHub.
# Runs from Windows Task Scheduler; OpenCode does not need to be open.
$ErrorActionPreference = "Stop"

$repo = Split-Path -Parent $PSScriptRoot
$logDir = Join-Path (Split-Path -Parent $repo) "sketch_CG_backups"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$log = Join-Path $logDir "weekly-backup.log"

if ((Test-Path $log) -and ((Get-Item $log).Length -gt 1MB)) {
    Move-Item -Force $log "$log.old"
}

function Write-Log($msg) {
    "$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')  $msg" | Out-File -Append -Encoding utf8 $log
}

function Invoke-Git([string[]]$gitArgs) {
    $oldEap = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        $out = & git @gitArgs 2>&1
        $code = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $oldEap
    }
    if ($out) { $out | Out-File -Append -Encoding utf8 $log }
    if ($code -ne 0) { throw "git $($gitArgs -join ' ') exited $code" }
    return $out
}

Set-Location $repo
$env:GIT_TERMINAL_PROMPT = "0"
$env:GCM_INTERACTIVE = "Never"
Write-Log "===== start ====="

try {
    Invoke-Git @("add", "-A") | Out-Null
    $changes = Invoke-Git @("status", "--porcelain")
    if ($changes) {
        Invoke-Git @("commit", "-m", "Auto-save $(Get-Date -Format 'yyyy-MM-dd HH:mm')") | Out-Null
        Write-Log "committed: $(@($changes).Count) file(s)"
    } else {
        Write-Log "no local changes"
    }

    # Pick up commits made on another computer, then send ours.
    Invoke-Git @("pull", "--rebase", "origin", "main") | Out-Null
    Invoke-Git @("push", "origin", "main") | Out-Null
    Write-Log "synced with origin/main"
} catch {
    Write-Log "ERROR: $_"
    if ((Test-Path (Join-Path $repo ".git\rebase-merge")) -or
        (Test-Path (Join-Path $repo ".git\rebase-apply"))) {
        $ErrorActionPreference = "Continue"
        & git rebase --abort 2>&1 | Out-File -Append -Encoding utf8 $log
    }
}
