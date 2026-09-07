<#
Apply-FanPatch.ps1 — 把风机控制 Tab 页集成到 D:\stm32\host 上位机项目
用法：在 PowerShell 里运行（必要时 -ExecutionPolicy Bypass）
    powershell -ExecutionPolicy Bypass -File Apply-FanPatch.ps1
#>

$ErrorActionPreference = "Stop"

$here     = Split-Path -Parent $MyInvocation.MyCommand.Path
$fanDir   = $here
$hostDir  = "D:\stm32\host"

Write-Host "=== Apply Fan Control Patch to Host App ===" -ForegroundColor Cyan
Write-Host "fan dir : $fanDir"
Write-Host "host dir: $hostDir"
Write-Host ""

if (-not (Test-Path $hostDir)) {
    throw "Host directory not found: $hostDir"
}

# 1) FanSerialClient.cs（新文件）
Write-Host "[1/4] FanSerialClient.cs -> host" -ForegroundColor Yellow
$src = Join-Path $fanDir "FanSerialClient.cs"
$dst = Join-Path $hostDir "FanSerialClient.cs"
Copy-Item -LiteralPath $src -Destination $dst -Force
Write-Host "  OK" -ForegroundColor Green

# 2) MainWindow.xaml
Write-Host "[2/4] MainWindow.xaml (patched copy -> host)" -ForegroundColor Yellow
$src = Join-Path $fanDir "_host_MainWindow.xaml"
$dst = Join-Path $hostDir "MainWindow.xaml"
Copy-Item -LiteralPath $src -Destination $dst -Force
Write-Host "  OK" -ForegroundColor Green

# 3) MainWindow.xaml.cs
Write-Host "[3/4] MainWindow.xaml.cs (patched copy -> host)" -ForegroundColor Yellow
$src = Join-Path $fanDir "_host_MainWindow.xaml.cs"
$dst = Join-Path $hostDir "MainWindow.xaml.cs"
Copy-Item -LiteralPath $src -Destination $dst -Force
Write-Host "  OK" -ForegroundColor Green

# 4) 清理临时文件（_host_*）
Write-Host "[4/4] Remove scratch _host_* files" -ForegroundColor Yellow
Get-ChildItem -LiteralPath $fanDir -Filter "_host_*" | Remove-Item -Force
Write-Host "  OK" -ForegroundColor Green

Write-Host ""
Write-Host "=== Patch applied. Now run dotnet build to verify ===" -ForegroundColor Cyan
Write-Host "  cd D:\stm32\host"
Write-Host "  dotnet build -c Debug"
Write-Host ""
Write-Host "To launch from command line:" -ForegroundColor Cyan
Write-Host "  dotnet run -c Debug --project GamepadSpeedController.csproj"
Write-Host "Or open GamepadSpeedController.csproj in Visual Studio and press F5."
