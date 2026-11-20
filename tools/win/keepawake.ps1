# Keeps Windows out of (Modern) Standby while training players (Throw.exe) or a batchmode Unity run.
# Uses SetThreadExecutionState from this process only: no power settings change, and it lapses when this exits,
# which it does 10 minutes after the last training player is gone.
Add-Type -Name Power -Namespace Win32 -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint esFlags);'
$ES_CONTINUOUS = [uint32]"0x80000000"; $ES_SYSTEM = [uint32]1; $ES_DISPLAY = [uint32]2
$idle = 0
while ($idle -lt 20) {
  if (Get-Process Throw, Unity -ErrorAction SilentlyContinue) { $idle = 0 } else { $idle++ }
  [void][Win32.Power]::SetThreadExecutionState($ES_CONTINUOUS -bor $ES_SYSTEM -bor $ES_DISPLAY)
  Start-Sleep -Seconds 30
}
[void][Win32.Power]::SetThreadExecutionState($ES_CONTINUOUS)
