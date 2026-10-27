param([string]$VcVars='D:\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat',
      [string]$Converter=(Join-Path $PSScriptRoot '../../STM32F407VET6黑匣子/tools/blackbox_to_csv.py'))
$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    New-Item -ItemType Directory tests/build -Force | Out-Null
    $common='/nologo /std:c11 /D_CRT_SECURE_NO_WARNINGS'
    function Run-Cl([string]$arguments,[string]$execute) {
        & cmd /d /s /c ('call "{0}" >nul && cl {1} {2} && {3}' -f $VcVars,$common,$arguments,$execute)
        if($LASTEXITCODE -ne 0){throw "Host test failed: $execute"}
    }
    Run-Cl 'tests/test_blackbox_crc.c /Fo:tests/build/test_blackbox_crc.o /Fe:tests/build/test_blackbox_crc.exe' 'tests\build\test_blackbox_crc.exe'
    Run-Cl '/Itests/sd_stubs /IApp/storage /IMiddlewares/Third_Party/FatFs/src tests/test_sd_spi.c /Fo:tests/build/test_sd_spi.o /Fe:tests/build/test_sd_spi.exe' 'tests\build\test_sd_spi.exe'
    Run-Cl '/Itests/sd_stubs /IApp/storage /IMiddlewares/Third_Party/FatFs/src tests/test_sd_short.c /Fo:tests/build/test_sd_short.o /Fe:tests/build/test_sd_short.exe' 'tests\build\test_sd_short.exe'
    Run-Cl '/Itests/sd_stubs /IApp/storage /IMiddlewares/Third_Party/FatFs/src tests/test_sd_bitbang.c /Fo:tests/build/test_sd_bitbang.o /Fe:tests/build/test_sd_bitbang.exe' 'tests\build\test_sd_bitbang.exe'
    Run-Cl '/Itests/sd_stubs tests/test_sd_eot.c /Fo:tests/build/test_sd_eot.o /Fe:tests/build/test_sd_eot.exe' 'tests\build\test_sd_eot.exe'
    & (Join-Path $PSScriptRoot 'run_sd_poll.ps1') -VcVars $VcVars
    Run-Cl '/DBB_HOST_TEST /DBB_FILE_BYTES=65536 /DBB_GROW_MARGIN=16384 /DBB_MAX_FILE_BYTES=4194304 /DBB_DIAGNOSTIC_MS=0 /Itests/pid_tuning/stubs /IApp /IApp/storage /IMiddlewares/Third_Party/FatFs/src tests/test_sd_storage.c Middlewares/Third_Party/FatFs/src/ff.c /Fo:tests/build/ /Fe:tests/build/test_sd_storage.exe' 'cmd /c exit 0'
    foreach($extra in @($false,$true)) {
        $name=if($extra){'extra'}else{'base'}
        $def=if($extra){'/DBLACKBOX_TEST_EXTRA_FIELDS'}else{''}
        Run-Cl "$def /Itests/pid_tuning/stubs tests/pid_tuning/test_blackbox.c /Fo:tests/build/sender_$name.o /Fe:tests/build/sender_$name.exe" "tests\build\sender_$name.exe tests\build\sd_$name.wire"
        & tests/build/test_sd_storage.exe "tests/build/sd_$name.wire" "tests/build/sd_$name.bin"
        if($LASTEXITCODE -ne 0){throw "SD storage test failed"}
        & python -B $Converter "tests/build/sd_$name.bin" -o "tests/build/sd_$name.csv" --report "tests/build/sd_$name.json"
        if($LASTEXITCODE -ne 0){throw "Converter failed"}
        $report=Get-Content "tests/build/sd_$name.json" -Raw | ConvertFrom-Json
        if($report.decoded_records -ne 2000 -or $report.undecoded_records -ne 0 -or $report.bad_data -ne 0 -or $report.bad_schema_fragments -ne 0){throw 'Round-trip mismatch'}
        & python -B tests/verify_yaw_blackbox.py @($report.outputs)
        if($LASTEXITCODE -ne 0){throw 'Yaw field round-trip mismatch'}
    }
    Run-Cl '/Itests/pid_tuning/stubs tests/pid_tuning/test_blackbox_schedule.c /Fo:tests/build/schedule.o /Fe:tests/build/schedule.exe' 'tests\build\schedule.exe'
    Run-Cl '/DBB_HOST_TEST /DBB_FILE_BYTES=41943040 /DBB_MAX_FILE_BYTES=67108864 /DBB_DIAGNOSTIC_MS=0 /Itests/pid_tuning/stubs /IApp /IApp/storage /IMiddlewares/Third_Party/FatFs/src tests/test_sd_storage.c Middlewares/Third_Party/FatFs/src/ff.c /Fo:tests/build/ /Fe:tests/build/reserve.exe' 'tests\build\reserve.exe tests\build\sd_base.wire tests\build\reserve.bin'
    Run-Cl '/Itests/pid_tuning/stubs tests/pid_tuning/test_bmi088_attitude.c /Fo:tests/build/attitude.o /Fe:tests/build/attitude.exe' 'tests\build\attitude.exe'
    Write-Output 'PASS SPI SD protocol, production queue/FatFs/recovery, unchanged converter with base/extended fields, 1kHz scheduler and BMI AHRS'
} finally { Pop-Location }
