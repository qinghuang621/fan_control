#Requires -Version 5.1
<#
fan_control_gui.ps1 - RoboMaster C-Board Fan Control GUI
Usage, pick one:
  (1) Right-click  -> Run with PowerShell
  (2) Command line: powershell -NoProfile -ExecutionPolicy Bypass -File fan_control_gui.ps1
Dependencies: Windows PowerShell 5.1 / .NET Framework 4.x (both built into Win10/11)
#>

[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
# SerialPort lives in System.dll (.NET Framework 4.x), so load System first.
Add-Type -AssemblyName System
try {
    # Try the full strong name (some PowerShell setups need this)
    Add-Type -AssemblyName "System.IO.Ports, Version=4.0.0.0, Culture=neutral, PublicKeyToken=cc7b13ffcd2ddd51" -ErrorAction SilentlyContinue
} catch {}

[System.Windows.Forms.Application]::EnableVisualStyles() | Out-Null

# =============================
# Main form
# =============================
$form = New-Object System.Windows.Forms.Form
$form.Text          = "C-Board Fan Control v1.0"
$form.Size          = New-Object System.Drawing.Size(760, 600)
$form.MinimumSize   = New-Object System.Drawing.Size(720, 560)
$form.StartPosition = "CenterScreen"
$fontUI  = New-Object System.Drawing.Font("Microsoft YaHei UI", 10)
$fontLog = New-Object System.Drawing.Font("Consolas", 9)
$fontBig = New-Object System.Drawing.Font("Consolas", 12, [System.Drawing.FontStyle]::Bold)
$form.Font = $fontUI

$ser = New-Object System.IO.Ports.SerialPort
$ser.ReadTimeout  = 200
$ser.WriteTimeout = 500
$rxQueue = [System.Collections.Queue]::new()

# =============================
# Serial data received -> enqueue for UI thread
# =============================
function Register-RxEvent {
    try { Unregister-Event -SourceIdentifier "ser_rx" -ErrorAction SilentlyContinue } catch {}
    Register-ObjectEvent -InputObject $ser -EventName DataReceived -SourceIdentifier "ser_rx" -Action {
        param($s, $e)
        try {
            do {
                $line = $null
                try   { $line = $s.ReadLine() } catch { return }
                if ($null -ne $line) {
                    $q = $Event.MessageData
                    [System.Threading.Monitor]::Enter($q)
                    try   { $q.Enqueue($line) } finally { [System.Threading.Monitor]::Exit($q) }
                }
            } while ($s.BytesToRead -gt 0)
        } catch {}
    } -MessageData $rxQueue | Out-Null
}

# =============================
# Helpers
# =============================
function Write-Log($msg, [System.Drawing.Color]$color = [System.Drawing.Color]::Black) {
    $ts = Get-Date -Format "HH:mm:ss"
    $txtLog.SelectionStart  = $txtLog.TextLength
    $txtLog.SelectionLength = 0
    $txtLog.SelectionColor  = $color
    $txtLog.AppendText("$ts $msg`r`n")
    $txtLog.ScrollToCaret()
}

function Send-Cmd($cmd) {
    if (-not $ser.IsOpen) {
        [void][MessageBox]::Show("Please connect the serial port first.", "Notice", 0, 48)
        return
    }
    try {
        $ser.WriteLine($cmd)
        Write-Log "-> $cmd" ([System.Drawing.Color]::RoyalBlue)
    } catch {
        Write-Log ("[Send failed] " + $_.Exception.Message) ([System.Drawing.Color]::Red)
    }
}

function Refresh-Ports {
    $cmbPort.Items.Clear()
    $names = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($names.Count -eq 0) { return }
    # 1) Collect friendly names via registry + WMI
    $map = @{}
    try {
        Get-CimInstance Win32_SerialPort -ErrorAction SilentlyContinue | ForEach-Object {
            if ($_.DeviceID) { $map[$_.DeviceID] = $_.Description }
        }
        Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match 'COM(\d+)' -and $_.ConfigManagerErrorCode -eq 0 } | ForEach-Object {
                $p = "COM" + $Matches[1]
                if (-not $map.ContainsKey($p)) { $map[$p] = $_.Name }
            }
    } catch {}
    foreach ($p in $names) {
        $display = if ($map.ContainsKey($p)) { "$p   $($map[$p])" } else { $p }
        [void]$cmbPort.Items.Add($display)
    }
    # Auto-select
    $pick = $null
    foreach ($it in $cmbPort.Items) {
        if ($it -match "STMicro|Virtual COM|USB Serial|USB.*Serial") { $pick = $it; break }
    }
    if (-not $pick) { $pick = $cmbPort.Items | Where-Object { $_ -like "*COM3*" } | Select-Object -First 1 }
    if (-not $pick -and $cmbPort.Items.Count -gt 0) { $pick = $cmbPort.Items[0] }
    if ($pick) { $cmbPort.SelectedItem = $pick }
}

function Update-FromStatus($line) {
    if ($line -match '^STATE=(\S+)\s+DUTY=(\d+)\s+TARGET=(\d+)') {
        $lblState.Text   = "State :  " + $Matches[1]
        $lblCurrent.Text = "Current:  " + $Matches[2] + " %"
        $lblTarget.Text  = "Target :  " + $Matches[3] + " %"
        try { $progDuty.Value = [int]$Matches[2] } catch {}
        return $true
    }
    if ($line -match '^OK S(\d+) TARGET=(\d+)') {
        $lblTarget.Text = "Target :  " + $Matches[2] + " %"
        return $true
    }
    if ($line -match '^OK START') { $lblState.Text = "State :  Starting"; return $true }
    if ($line -match '^OK STOP')  { $lblState.Text = "State :  Stopping"; return $true }
    return $false
}

function Set-Controls($enabled) {
    $btnStart.Enabled    = $enabled
    $btnStop.Enabled     = $enabled
    $btnStatus.Enabled   = $enabled
    $btnSetDuty.Enabled  = $enabled
    $btnSendCmd.Enabled  = $enabled
    $trkDuty.Enabled     = $enabled
    $entCmd.Enabled      = $enabled
    foreach ($b in $script:presetBtns) { $b.Enabled = $enabled }
}

# =============================
# Group 1: Connection
# =============================
$grpConn = New-Object System.Windows.Forms.GroupBox
$grpConn.Text = "  Serial Connection  "
$grpConn.Location = New-Object System.Drawing.Point(12, 12)
$grpConn.Size     = New-Object System.Drawing.Size(720, 74)
$form.Controls.Add($grpConn)

$lblPort = New-Object System.Windows.Forms.Label
$lblPort.Text = "Port:"; $lblPort.Location = New-Object System.Drawing.Point(16, 30); $lblPort.AutoSize = $true
$grpConn.Controls.Add($lblPort)

$cmbPort = New-Object System.Windows.Forms.ComboBox
$cmbPort.Location = New-Object System.Drawing.Point(60, 26); $cmbPort.Size = New-Object System.Drawing.Size(320, 28)
$cmbPort.DropDownStyle = "DropDownList"
$grpConn.Controls.Add($cmbPort)

$lblBaud = New-Object System.Windows.Forms.Label
$lblBaud.Text = "Baud:"; $lblBaud.Location = New-Object System.Drawing.Point(394, 30); $lblBaud.AutoSize = $true
$grpConn.Controls.Add($lblBaud)

$cmbBaud = New-Object System.Windows.Forms.ComboBox
$cmbBaud.Location = New-Object System.Drawing.Point(438, 26); $cmbBaud.Size = New-Object System.Drawing.Size(100, 28)
$cmbBaud.DropDownStyle = "DropDownList"
@(9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600) | ForEach-Object { [void]$cmbBaud.Items.Add($_) }
$cmbBaud.SelectedItem = 115200
$grpConn.Controls.Add($cmbBaud)

$btnRefresh = New-Object System.Windows.Forms.Button
$btnRefresh.Text = "Refresh"; $btnRefresh.Location = New-Object System.Drawing.Point(550, 25)
$btnRefresh.Size = New-Object System.Drawing.Size(75, 30)
$btnRefresh.Add_Click({ Refresh-Ports })
$grpConn.Controls.Add($btnRefresh)

$btnConn = New-Object System.Windows.Forms.Button
$btnConn.Text = "Connect"; $btnConn.Location = New-Object System.Drawing.Point(632, 25)
$btnConn.Size = New-Object System.Drawing.Size(75, 30)
$btnConn.BackColor = [System.Drawing.Color]::PaleGreen
$grpConn.Controls.Add($btnConn)

$lblConn = New-Object System.Windows.Forms.Label
$lblConn.Text = "Disconnected"; $lblConn.ForeColor = [System.Drawing.Color]::Gray
$lblConn.Location = New-Object System.Drawing.Point(16, 54); $lblConn.AutoSize = $true
$grpConn.Controls.Add($lblConn)

# =============================
# Group 2: Fan Control
# =============================
$grpCtrl = New-Object System.Windows.Forms.GroupBox
$grpCtrl.Text = "  Fan Control  "
$grpCtrl.Location = New-Object System.Drawing.Point(12, 96)
$grpCtrl.Size     = New-Object System.Drawing.Size(720, 178)
$form.Controls.Add($grpCtrl)

$btnStart = New-Object System.Windows.Forms.Button
$btnStart.Text = "  START"
$btnStart.Font = New-Object System.Drawing.Font("Microsoft YaHei UI", 10, [System.Drawing.FontStyle]::Bold)
$btnStart.Location = New-Object System.Drawing.Point(14, 32); $btnStart.Size = New-Object System.Drawing.Size(160, 36)
$btnStart.Enabled = $false
$btnStart.Add_Click({ Send-Cmd "START" })
$grpCtrl.Controls.Add($btnStart)

$btnStop = New-Object System.Windows.Forms.Button
$btnStop.Text = "  STOP"
$btnStop.Font = New-Object System.Drawing.Font("Microsoft YaHei UI", 10, [System.Drawing.FontStyle]::Bold)
$btnStop.ForeColor = [System.Drawing.Color]::DarkRed
$btnStop.Location = New-Object System.Drawing.Point(184, 32); $btnStop.Size = New-Object System.Drawing.Size(160, 36)
$btnStop.Enabled = $false
$btnStop.Add_Click({ Send-Cmd "STOP" })
$grpCtrl.Controls.Add($btnStop)

$btnStatus = New-Object System.Windows.Forms.Button
$btnStatus.Text = "Refresh Status"
$btnStatus.Location = New-Object System.Drawing.Point(354, 32); $btnStatus.Size = New-Object System.Drawing.Size(110, 36)
$btnStatus.Enabled = $false
$btnStatus.Add_Click({ Send-Cmd "STATUS" })
$grpCtrl.Controls.Add($btnStatus)

$lblDutyHint = New-Object System.Windows.Forms.Label
$lblDutyHint.Text = "Target duty (drag slider, then press Set):"
$lblDutyHint.Location = New-Object System.Drawing.Point(14, 84); $lblDutyHint.AutoSize = $true
$grpCtrl.Controls.Add($lblDutyHint)

$trkDuty = New-Object System.Windows.Forms.TrackBar
$trkDuty.Minimum = 0; $trkDuty.Maximum = 100; $trkDuty.Value = 50; $trkDuty.TickFrequency = 5
$trkDuty.Location = New-Object System.Drawing.Point(14, 106); $trkDuty.Size = New-Object System.Drawing.Size(500, 45)
$trkDuty.Enabled = $false
$trkDuty.Add_ValueChanged({ $lblDutyVal.Text = "$($trkDuty.Value) %" })
$grpCtrl.Controls.Add($trkDuty)

$lblDutyVal = New-Object System.Windows.Forms.Label
$lblDutyVal.Text = "50 %"; $lblDutyVal.Font = $fontBig
$lblDutyVal.Location = New-Object System.Drawing.Point(520, 110); $lblDutyVal.Size = New-Object System.Drawing.Size(74, 28)
$lblDutyVal.TextAlign = [System.Drawing.ContentAlignment]::MiddleRight
$grpCtrl.Controls.Add($lblDutyVal)

$btnSetDuty = New-Object System.Windows.Forms.Button
$btnSetDuty.Text = "Set"
$btnSetDuty.Location = New-Object System.Drawing.Point(600, 108); $btnSetDuty.Size = New-Object System.Drawing.Size(104, 32)
$btnSetDuty.Enabled = $false
$btnSetDuty.Add_Click({ Send-Cmd ("S" + $trkDuty.Value) })
$grpCtrl.Controls.Add($btnSetDuty)

$lblPreset = New-Object System.Windows.Forms.Label
$lblPreset.Text = "Presets:"
$lblPreset.Location = New-Object System.Drawing.Point(14, 148); $lblPreset.AutoSize = $true
$grpCtrl.Controls.Add($lblPreset)

$script:presetBtns = @()
$x = 90
foreach ($pct in @(10, 25, 30, 50, 70, 80, 90, 100)) {
    $b = New-Object System.Windows.Forms.Button
    $b.Text = "${pct}%"; $b.Location = New-Object System.Drawing.Point($x, 142)
    $b.Size = New-Object System.Drawing.Size(56, 28); $b.Enabled = $false
    $b.Tag = $pct
    $b.Add_Click({
        $p = [int]$this.Tag
        $trkDuty.Value = $p
        Send-Cmd ("S" + $p)
    })
    $grpCtrl.Controls.Add($b)
    $script:presetBtns += $b
    $x += 64
}

# =============================
# Group 3: Current Status
# =============================
$grpState = New-Object System.Windows.Forms.GroupBox
$grpState.Text = "  Current Status  "
$grpState.Location = New-Object System.Drawing.Point(12, 284)
$grpState.Size     = New-Object System.Drawing.Size(720, 82)
$form.Controls.Add($grpState)

$lblState = New-Object System.Windows.Forms.Label
$lblState.Text = "State :  --"; $lblState.Font = $fontBig
$lblState.Location = New-Object System.Drawing.Point(16, 30); $lblState.AutoSize = $true
$grpState.Controls.Add($lblState)

$lblCurrent = New-Object System.Windows.Forms.Label
$lblCurrent.Text = "Current:  --"; $lblCurrent.Font = $fontBig
$lblCurrent.Location = New-Object System.Drawing.Point(240, 30); $lblCurrent.AutoSize = $true
$grpState.Controls.Add($lblCurrent)

$lblTarget = New-Object System.Windows.Forms.Label
$lblTarget.Text = "Target :  --"; $lblTarget.Font = $fontBig
$lblTarget.Location = New-Object System.Drawing.Point(470, 30); $lblTarget.AutoSize = $true
$grpState.Controls.Add($lblTarget)

$progDuty = New-Object System.Windows.Forms.ProgressBar
$progDuty.Minimum = 0; $progDuty.Maximum = 100; $progDuty.Value = 0
$progDuty.Location = New-Object System.Drawing.Point(16, 56); $progDuty.Size = New-Object System.Drawing.Size(688, 18)
$grpState.Controls.Add($progDuty)

# =============================
# Group 4: Log + Manual Cmd
# =============================
$grpLog = New-Object System.Windows.Forms.GroupBox
$grpLog.Text = "  Serial Log  /  Manual command (Enter to send)  "
$grpLog.Location = New-Object System.Drawing.Point(12, 376)
$grpLog.Size     = New-Object System.Drawing.Size(720, 178)
$form.Controls.Add($grpLog)

$txtLog = New-Object System.Windows.Forms.RichTextBox
$txtLog.Font = $fontLog
$txtLog.Location = New-Object System.Drawing.Point(10, 24)
$txtLog.Size     = New-Object System.Drawing.Size(700, 118)
$txtLog.ReadOnly = $true
$grpLog.Controls.Add($txtLog)

$entCmd = New-Object System.Windows.Forms.TextBox
$entCmd.Location = New-Object System.Drawing.Point(10, 148); $entCmd.Size = New-Object System.Drawing.Size(588, 28)
$entCmd.Enabled = $false
$entCmd.Add_KeyDown({
    if ($_.KeyCode -eq [System.Windows.Forms.Keys]::Enter) {
        $_.SuppressKeyPress = $true
        $c = $entCmd.Text.Trim()
        if ($c) { $entCmd.Text = ""; Send-Cmd $c }
    }
})
$grpLog.Controls.Add($entCmd)

$btnSendCmd = New-Object System.Windows.Forms.Button
$btnSendCmd.Text = "Send"; $btnSendCmd.Location = New-Object System.Drawing.Point(606, 146)
$btnSendCmd.Size = New-Object System.Drawing.Size(104, 28); $btnSendCmd.Enabled = $false
$btnSendCmd.Add_Click({
    $c = $entCmd.Text.Trim()
    if ($c) { $entCmd.Text = ""; Send-Cmd $c }
})
$grpLog.Controls.Add($btnSendCmd)

# =============================
# Connect / Disconnect handler
# =============================
$btnConn.Add_Click({
    if ($ser.IsOpen) {
        # ---- DISCONNECT ----
        try { Unregister-Event -SourceIdentifier "ser_rx" -ErrorAction SilentlyContinue } catch {}
        try { $ser.Close() } catch {}
        $btnConn.Text      = "Connect"
        $btnConn.BackColor = [System.Drawing.Color]::PaleGreen
        $lblConn.Text      = "Disconnected"
        $lblConn.ForeColor = [System.Drawing.Color]::Gray
        Set-Controls $false
        Write-Log "[Disconnected]" ([System.Drawing.Color]::Gray)
    } else {
        # ---- CONNECT ----
        if (-not $cmbPort.SelectedItem) {
            [void][MessageBox]::Show("Please select a COM port first.", "Notice", 0, 48)
            return
        }
        $portName = ($cmbPort.SelectedItem.ToString() -split "\s+", 2)[0]
        try {
            $ser.PortName  = $portName
            $ser.BaudRate  = [int]$cmbBaud.SelectedItem
            $ser.NewLine   = "`n"
            $ser.Open()
        } catch {
            [void][MessageBox]::Show(("Failed to open port: " + $_.Exception.Message), "Error", 0, 16)
            return
        }
        Register-RxEvent
        $btnConn.Text      = "Disconnect"
        $btnConn.BackColor = [System.Drawing.Color]::MistyRose
        $lblConn.Text      = "Connected  ($portName)"
        $lblConn.ForeColor = [System.Drawing.Color]::SeaGreen
        Set-Controls $true
        Write-Log ("[Connected] " + $portName) ([System.Drawing.Color]::SeaGreen)
        Start-Sleep -Milliseconds 180
        Send-Cmd "STATUS"
    }
})

# =============================
# Timer: drain received queue onto UI
# =============================
$tmr = New-Object System.Windows.Forms.Timer
$tmr.Interval = 25
$tmr.Add_Tick({
    $loops = 0
    while ($rxQueue.Count -gt 0 -and $loops -lt 16) {
        $line = $null
        [System.Threading.Monitor]::Enter($rxQueue)
        try   { if ($rxQueue.Count -gt 0) { $line = $rxQueue.Dequeue() } }
        finally { [System.Threading.Monitor]::Exit($rxQueue) }
        if ($null -eq $line) { break }
        $parsed = Update-FromStatus $line
        $color  = if ($parsed) { [System.Drawing.Color]::ForestGreen } else { [System.Drawing.Color]::DimGray }
        Write-Log ("<- " + $line) $color
        $loops++
    }
})
$tmr.Start()

# =============================
# Form closing: cleanup
# =============================
$form.Add_FormClosing({
    $tmr.Stop()
    try { Unregister-Event -SourceIdentifier "ser_rx" -ErrorAction SilentlyContinue } catch {}
    if ($ser.IsOpen) { try { $ser.Close() } catch {} }
    try { $ser.Dispose() } catch {}
})

# =============================
# Init & show
# =============================
Refresh-Ports
Set-Controls $false
[void]$form.ShowDialog()

# Final cleanup (after the dialog closes)
try { Unregister-Event -SourceIdentifier "ser_rx" -ErrorAction SilentlyContinue } catch {}
if ($ser.IsOpen) { try { $ser.Close() } catch {} }
try { $ser.Dispose() } catch {}
try { $tmr.Stop(); $tmr.Dispose() } catch {}
