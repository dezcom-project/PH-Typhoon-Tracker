# ----------------------------------------------------------------------------
# gen_map.ps1 - Rasterizes simplified Philippines coastline polygons into a
# 128x64 monochrome bitmap for Adafruit_GFX drawBitmap() (MSB = leftmost px).
#
# PAR mapping used by both the firmware and this script:
#   lon 116.0E..127.0E -> x 0..127   : x = (lon - 116) * 127 / 11
#   lat  21.5N..4.5N  -> y 63..0    : y = (21.5 - lat) * 63 / 17
#
# Output: ph_map_bitmap.txt (C hex array) + ASCII preview on stdout.
# ----------------------------------------------------------------------------

$ErrorActionPreference = 'Stop'

# Polygons as lists of @(lat, lon). Simplified coastlines (~0.1 deg detail,
# well below the ~0.09 deg/px resolution of the display).
$islands = @(
    @{ name='LUZON'; pts=@(
        @(18.60,120.75),@(18.55,121.00),@(18.36,121.64),@(18.46,122.14),
        @(17.80,122.10),@(16.90,122.20),@(16.35,122.15),@(15.80,121.60),
        @(14.74,121.65),@(14.19,121.73),@(14.15,122.20),@(14.11,122.95),
        @(13.70,123.55),@(13.14,123.73),@(12.97,124.05),@(12.67,123.88),
        @(12.90,123.55),@(13.30,123.30),@(13.95,122.95),@(13.92,122.10),
        @(13.55,122.15),@(13.35,122.10),@(13.60,121.90),@(13.93,121.61),
        @(13.75,121.05),@(14.20,120.90),@(14.55,120.60),@(14.67,120.28),
        @(15.20,120.05),@(15.90,119.90),@(16.33,119.89),@(16.03,120.23),
        @(16.62,120.32),@(17.57,120.39),@(18.20,120.55) )},
    @{ name='MINDORO'; pts=@(
        @(13.50,120.55),@(13.45,121.30),@(13.20,121.45),@(12.60,121.45),
        @(12.25,121.10),@(12.30,120.60),@(12.85,120.50) )},
    @{ name='PALAWAN'; pts=@(
        @(11.15,119.40),@(10.80,119.30),@(10.00,118.90),@(9.70,118.50),
        @(9.00,118.00),@(8.30,117.30),@(7.95,117.15),@(8.20,117.00),
        @(8.90,117.60),@(9.60,118.20),@(10.30,118.85),@(11.00,119.25) )},
    @{ name='MARINDUQUE'; pts=@(
        @(13.45,121.95),@(13.35,122.20),@(13.15,122.05),@(13.30,121.90) )},
    @{ name='POLILLO'; pts=@(
        @(14.85,121.90),@(14.75,122.20),@(14.60,122.05) )},
    @{ name='TABLAS'; pts=@(
        @(12.60,122.00),@(12.40,122.30),@(12.20,122.05) )},
    @{ name='MASBATE'; pts=@(
        @(12.55,123.60),@(12.30,124.00),@(12.00,123.80),@(12.20,123.40) )},
    @{ name='CATANDUANES'; pts=@(
        @(13.95,124.20),@(13.70,124.45),@(13.50,124.25),@(13.75,124.05) )},
    @{ name='PANAY'; pts=@(
        @(11.80,122.00),@(11.60,122.90),@(11.00,123.05),@(10.40,122.60),
        @(10.70,122.00),@(11.30,121.90) )},
    @{ name='NEGROS'; pts=@(
        @(10.90,122.90),@(10.50,123.30),@(9.40,123.30),@(9.00,122.70),
        @(9.60,122.30),@(10.50,122.50) )},
    @{ name='SIQUIJOR'; pts=@(
        @(9.25,123.55),@(9.15,123.75),@(9.00,123.60) )},
    @{ name='CEBU'; pts=@(
        @(11.30,123.90),@(10.90,124.10),@(9.90,124.00),@(9.70,123.60),
        @(10.70,123.55) )},
    @{ name='BOHOL'; pts=@(
        @(10.10,124.00),@(10.05,124.60),@(9.60,124.60),@(9.55,124.10) )},
    @{ name='SAMAR'; pts=@(
        @(12.60,124.30),@(12.55,125.00),@(12.00,125.55),@(11.00,125.60),
        @(10.90,125.00),@(11.30,124.80),@(11.20,124.40) )},
    @{ name='LEYTE'; pts=@(
        @(11.60,124.30),@(11.40,125.00),@(10.80,125.20),@(10.20,125.10),
        @(10.00,124.90),@(10.30,124.40),@(10.90,124.30) )},
    @{ name='MINDANAO'; pts=@(
        @(8.60,123.34),@(8.20,123.85),@(8.23,124.25),@(8.48,124.65),
        @(9.00,125.00),@(9.78,125.50),@(9.36,126.20),@(8.35,126.35),
        @(7.80,126.40),@(6.30,126.30),@(6.95,125.60),@(6.90,125.45),
        @(6.10,125.20),@(5.55,125.35),@(5.90,124.90),@(6.50,124.40),
        @(7.40,124.05),@(7.70,123.60),@(7.30,122.90),@(6.95,122.10),
        @(7.60,122.30),@(8.20,122.90) )},
    @{ name='BASILAN'; pts=@(
        @(6.90,121.95),@(6.85,122.25),@(6.55,122.20),@(6.60,121.95) )},
    @{ name='JOLO'; pts=@(
        @(6.15,120.90),@(6.05,121.20),@(5.75,121.15),@(5.85,120.85) )},
    @{ name='TAWITAWI'; pts=@(
        @(5.20,119.90),@(5.05,120.10),@(4.90,119.95),@(5.00,119.75) )},
    @{ name='BATANES'; pts=@(
        @(20.35,121.95),@(20.25,122.10),@(20.10,121.90) )}
)

function Test-InPolygon {
    param([double[]]$lats, [double[]]$lons, [double]$lat, [double]$lon)
    # Even-odd ray casting rule.
    $inside = $false
    $n = $lats.Count
    $j = $n - 1
    for ($i = 0; $i -lt $n; $i++) {
        if ((($lats[$i] -gt $lat) -ne ($lats[$j] -gt $lat)) -and
            ($lon -lt ($lons[$j] - $lons[$i]) * ($lat - $lats[$i]) / ($lats[$j] - $lats[$i]) + $lons[$i])) {
            $inside = -not $inside
        }
        $j = $i
    }
    return $inside
}

$W = 128; $H = 64
$LON_MIN = 116.0; $LON_MAX = 127.0
$LAT_MAX = 21.5;  $LAT_MIN = 4.5

# Build boolean land mask (pixel centers tested against every polygon).
$mask = New-Object 'bool[,]' $W,$H
foreach ($isl in $islands) {
    $lats = @($isl.pts | ForEach-Object { $_[0] })
    $lons = @($isl.pts | ForEach-Object { $_[1] })
    for ($y = 0; $y -lt $H; $y++) {
        $lat = $LAT_MAX - ($y + 0.5) * ($LAT_MAX - $LAT_MIN) / $H
        for ($x = 0; $x -lt $W; $x++) {
            $lon = $LON_MIN + ($x + 0.5) * ($LON_MAX - $LON_MIN) / $W
            if (Test-InPolygon $lats $lons $lat $lon) { $mask[$x,$y] = $true }
        }
    }
}

# ASCII preview so a human can sanity-check the shape before flashing.
Write-Output "ASCII PREVIEW (128x64):"
for ($y = 0; $y -lt $H; $y++) {
    $row = ''
    for ($x = 0; $x -lt $W; $x++) { if ($mask[$x,$y]) { $row += '#' } else { $row += '.' } }
    Write-Output $row
}

# Pack into bytes: MSB of each byte = leftmost pixel (Adafruit GFX convention).
$bytes = New-Object 'byte[]' ($W/8*$H)
for ($y = 0; $y -lt $H; $y++) {
    for ($x = 0; $x -lt $W; $x++) {
        if ($mask[$x,$y]) {
            $bytes[$y*($W/8) + [math]::Floor($x/8)] = $bytes[$y*($W/8) + [math]::Floor($x/8)] -bor (0x80 -shr ($x % 8))
        }
    }
}

$out = New-Object System.Collections.Generic.List[string]
for ($i = 0; $i -lt $bytes.Count; $i += 12) {
    $chunk = $bytes[$i..([Math]::Min($i+11, $bytes.Count-1))] | ForEach-Object { '0x{0:X2}' -f $_ }
    $out.Add(("  " + ($chunk -join ', ') + ","))
}
$bitmapText = $out -join "`r`n"
Set-Content -LiteralPath (Join-Path $PSScriptRoot 'ph_map_bitmap.txt') -Value $bitmapText -Encoding ASCII
Write-Output ""
Write-Output ("Wrote {0} bytes to ph_map_bitmap.txt" -f $bytes.Count)
