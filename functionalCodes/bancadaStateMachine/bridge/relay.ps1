# Relay de uma porta COM do Windows para um socket TCP no WSL (o WSL não enxerga portas COM).
# Iniciado automaticamente pelo bridge/server.mjs; sai quando o bridge fecha a conexão, para não
# deixar a porta COM presa. Uso manual:
#   powershell -ExecutionPolicy Bypass -File bridge\relay.ps1 -Port COM13 -Baud 115200 -TcpPort 8771
param([string]$Port = "COM13", [int]$Baud = 115200, [string]$TcpHost = "127.0.0.1", [int]$TcpPort = 8771)

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.DtrEnable = $false; $sp.RtsEnable = $false; $sp.ReadTimeout = 50
try { $sp.Open() } catch { Write-Host "[relay $Port] erro ao abrir: $($_.Exception.Message)"; exit 2 }
Write-Host "[relay $Port] aberto a $Baud"

$tcp = New-Object System.Net.Sockets.TcpClient
$tcp.NoDelay = $true
try { $tcp.Connect($TcpHost, $TcpPort) } catch { Write-Host "[relay $Port] sem bridge em ${TcpHost}:${TcpPort}"; $sp.Close(); exit 3 }
$ns = $tcp.GetStream()
Write-Host "[relay $Port] conectado ao bridge ${TcpHost}:${TcpPort}"

$buf = New-Object byte[] 4096
try {
  while ($tcp.Connected) {
    $idle = $true
    if ($sp.BytesToRead -gt 0) { $n = $sp.Read($buf, 0, $buf.Length); $ns.Write($buf, 0, $n); $idle = $false }
    if ($ns.DataAvailable) { $n = $ns.Read($buf, 0, $buf.Length); if ($n -le 0) { break }; $sp.Write($buf, 0, $n); $idle = $false }
    if ($tcp.Client.Poll(0, [System.Net.Sockets.SelectMode]::SelectRead) -and $tcp.Client.Available -eq 0) { break }
    if ($idle) { Start-Sleep -Milliseconds 5 }
  }
} catch { Write-Host "[relay $Port] $($_.Exception.Message)" }
try { $tcp.Close() } catch {}
$sp.Close()
Write-Host "[relay $Port] encerrado"
