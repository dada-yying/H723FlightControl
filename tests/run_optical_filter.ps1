param([string]$VcVars='D:\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat',
      [string]$ReplayDirectory='')
$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    $output='tests\build\optical_filter'
    New-Item -ItemType Directory $output -Force | Out-Null
    foreach($name in @('test_optical_flow','test_optical_diag','test_optical_filter','test_blackbox','test_blackbox_schedule')) {
        $command='call "{0}" >nul && cl /nologo /TC /std:c11 /D_CRT_SECURE_NO_WARNINGS /Itests/pid_tuning/stubs tests/pid_tuning/{1}.c /Fo:{2}\{1}.o /Fe:{2}\{1}.exe && {2}\{1}.exe {2}\{1}.wire' -f $VcVars,$name,$output
        & cmd /d /s /c $command
        if($LASTEXITCODE -ne 0){throw "$name failed"}
    }
    $command='call "{0}" >nul && cl /nologo /TC /std:c11 /D_CRT_SECURE_NO_WARNINGS /DBLACKBOX_TEST_EXTRA_FIELDS /Itests/pid_tuning/stubs tests/pid_tuning/test_blackbox.c /Fo:{1}\extra.o /Fe:{1}\extra.exe && {1}\extra.exe {1}\extra.wire' -f $VcVars,$output
    & cmd /d /s /c $command
    if($LASTEXITCODE -ne 0){throw 'Extended blackbox failed'}
    if($ReplayDirectory) {
        & python -B tests/verify_optical_filter_replay.py --data $ReplayDirectory --exe "$output/test_optical_filter.exe" --output $output
        if($LASTEXITCODE -ne 0){throw 'Flight log replay failed'}
    }
} finally { Pop-Location }
