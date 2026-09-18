param([string]$OutputDirectory = 'ws2812-hal-verified')
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    $bin = 'D:\keil5\ARM\ARMCC\Bin'
    $outDir = $OutputDirectory
    New-Item -ItemType Directory -Path $outDir -Force | Out-Null
    Start-Transcript -Path "$outDir/build.log" -Force | Out-Null
    try {
        $project = [xml](Get-Content -Raw 'CtrlBoard-H7.uvprojx')
        $ctl = $project.Project.Targets.Target.TargetOption.TargetArmAds.Cads.VariousControls
        $ccArgs = @('--cpu=Cortex-M7.fp.dp', '--c99', '--gnu', '-O3', '--split_sections', '-g', '-c')
        foreach ($inc in $ctl.IncludePath.Split(';')) { $ccArgs += @('-I', $inc) }
        foreach ($def in $ctl.Define.Split(',')) { $ccArgs += @('-D', $def) }
        $objects = @()
        foreach ($file in $project.SelectNodes('//Groups/Group/Files/File')) {
            $src = [string]$file.FilePath
            $obj = Join-Path $outDir ([IO.Path]::GetFileNameWithoutExtension($src) + '.o')
            if ($file.FileType -eq '1') {
                & "$bin/armcc.exe" @ccArgs $src -o $obj
            } elseif ($file.FileType -eq '2') {
                & "$bin/armasm.exe" --cpu Cortex-M7.fp.dp -g $src -o $obj
            } else { continue }
            if ($LASTEXITCODE -ne 0) { throw "Compile failed: $src" }
            $objects += $obj
        }
        $scatter = [string]$project.Project.Targets.Target.TargetOption.TargetArmAds.LDads.ScatterFile
        if ([string]::IsNullOrWhiteSpace($scatter)) { $scatter = 'CtrlBoard-H7/CtrlBoard-H7.sct' }
        & "$bin/armlink.exe" --cpu Cortex-M7.fp.dp @objects --scatter $scatter --strict --map --symbols --xref --callgraph --list "$outDir/firmware.map" -o "$outDir/firmware.axf"
        if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
        & "$bin/fromelf.exe" --i32combined --output "$outDir/firmware.hex" "$outDir/firmware.axf"
        if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
        Write-Output "BUILD_PASS: $($objects.Count) source files"
        Get-FileHash "$outDir/firmware.hex" | Format-List
    } finally { Stop-Transcript | Out-Null }
} finally { Pop-Location }
