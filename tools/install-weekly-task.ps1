# Register weekly auto-save (Sunday 12:00).
# Run once: right-click -> "Run with PowerShell".
# If "Access is denied", run PowerShell as Administrator.
$taskName = "CG_Scale weekly backup"
$script = Join-Path $PSScriptRoot "weekly-backup.ps1"

$arg = '-NoProfile -ExecutionPolicy Bypass -File "{0}"' -f $script
$action = New-ScheduledTaskAction -Execute "powershell.exe" -Argument $arg
$trigger = New-ScheduledTaskTrigger -Weekly -DaysOfWeek Sunday -At 12:00
$settings = New-ScheduledTaskSettingsSet -StartWhenAvailable -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -MultipleInstances IgnoreNew -ExecutionTimeLimit (New-TimeSpan -Minutes 10)

Register-ScheduledTask -TaskName $taskName -Action $action -Trigger $trigger -Settings $settings -Description "Git commit + push CG_Scale weekly" -Force

Write-Host "Done: task '$taskName' created (Sunday 12:00)."
