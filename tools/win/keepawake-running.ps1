# Exit 0 when the keep-awake helper is running.
if (Get-CimInstance Win32_Process | Where-Object { $_.Name -eq "powershell.exe" -and $_.CommandLine -like "*keepawake.ps1*" }) { exit 0 } else { exit 1 }
