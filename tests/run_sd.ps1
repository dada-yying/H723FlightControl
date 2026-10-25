param([string]$VcVars='D:\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat',
      [string]$OutputDirectory='', [string]$ReceiverDirectory='')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'host_test_helpers.ps1')
if(!$OutputDirectory) { $OutputDirectory=Join-Path $PSScriptRoot ('build/sd-'+[guid]::NewGuid().ToString('N')) }
if(!$ReceiverDirectory) { $ReceiverDirectory=Join-Path $PSScriptRoot '../../references/STM32F407VET6黑匣子(弃用）' }
$suite=New-HostSuite 'spi_sd' $OutputDirectory
$compilerProblem=if(!(Test-Path -LiteralPath $VcVars)){'MSVC vcvars64.bat not found: '+$VcVars}else{''}
$converter=Join-Path $ReceiverDirectory 'tools/blackbox_to_csv.py'
$converterProblem=if(!(Test-Path -LiteralPath $converter)){'Converter not found: '+$converter}elseif(!(Get-Command python -ErrorAction SilentlyContinue)){'Python not found'}else{''}
Push-Location (Join-Path $PSScriptRoot '..')
try {
    # Existing polling fixture includes this generated file by its fixed path.
    New-Item -ItemType Directory tests/build -Force | Out-Null
    $src=[IO.File]::ReadAllText((Join-Path (Get-Location) 'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c'))
    $start=$src.IndexOf('HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,')
    $end=$src.IndexOf('HAL_StatusTypeDef HAL_SPI_Transmit_IT(',$start)
    $pollProblem=if($start -lt 0 -or $end -le $start){'Production HAL polling function not found'}else{''}
    if(!$pollProblem) { [IO.File]::WriteAllText((Join-Path (Get-Location) 'tests/build/spi_poll_body.inc'),$src.Substring($start,$end-$start)) }
    $common='/Itests/sd_stubs /IApp/storage /IMiddlewares/Third_Party/FatFs/src'
    foreach($name in @('test_blackbox_crc','test_sd_spi','test_sd_short','test_sd_bitbang','test_sd_eot','test_sd_poll_hal')) {
        $compile='call "{0}" >nul && cl /nologo /TC /std:c11 /D_CRT_SECURE_NO_WARNINGS {1} tests/{2}.c /Fo:"{3}/{2}.o" /Fe:"{3}/{2}.exe"' -f $VcVars,$common,$name,$suite.output
        $problem=if($compilerProblem){$compilerProblem}elseif($name -eq 'test_sd_poll_hal'){$pollProblem}else{''}
        $null=Invoke-HostCase $suite $name $compile ('"{0}/{1}.exe"' -f $suite.output,$name) -EnvironmentProblem $problem
    }
    $storageFlags='/DBB_HOST_TEST /DBB_FILE_BYTES=65536 /DBB_GROW_MARGIN=16384 /DBB_MAX_FILE_BYTES=4194304 /DBB_DIAGNOSTIC_MS=0 /Itests/pid_tuning/stubs /IApp /IApp/storage /IMiddlewares/Third_Party/FatFs/src'
    $compile='call "{0}" >nul && cl /nologo /std:c11 /D_CRT_SECURE_NO_WARNINGS {1} tests/test_sd_storage.c Middlewares/Third_Party/FatFs/src/ff.c /Fo:"{2}/" /Fe:"{2}/test_sd_storage.exe"' -f $VcVars,$storageFlags,$suite.output
    $storageReady=Invoke-HostCase $suite 'storage_compile' $compile '' -EnvironmentProblem $compilerProblem
    $baseReady=$false
    foreach($variant in @('base','extra')) {
        $define=if($variant -eq 'extra'){'/DBLACKBOX_TEST_EXTRA_FIELDS'}else{''}
        $compile='call "{0}" >nul && cl /nologo /TC /std:c11 /D_CRT_SECURE_NO_WARNINGS {1} /Itests/pid_tuning/stubs tests/pid_tuning/test_blackbox.c /Fo:"{2}/sender_{3}.o" /Fe:"{2}/sender_{3}.exe"' -f $VcVars,$define,$suite.output,$variant
        $senderReady=Invoke-HostCase $suite "sender_$variant" $compile ('"{0}/sender_{1}.exe" "{0}/sd_{1}.wire"' -f $suite.output,$variant) -EnvironmentProblem $compilerProblem
        $logReady=Invoke-HostCase $suite "storage_$variant" '' ('"{0}/test_sd_storage.exe" "{0}/sd_{1}.wire" "{0}/sd_{1}.bin"' -f $suite.output,$variant) -Prerequisite ($storageReady -and $senderReady)
        $converted=Invoke-HostCase $suite "convert_$variant" '' ('python -B "{0}" "{1}/sd_{2}.bin" -o "{1}/sd_{2}.csv" --report "{1}/sd_{2}.json"' -f $converter,$suite.output,$variant) -Prerequisite $logReady -EnvironmentProblem $converterProblem
        $check='python -B -c "import json; r=json.load(open(r''{0}/sd_{1}.json'')); assert r[''decoded_records'']==2000 and r[''undecoded_records'']==r[''bad_data'']==r[''bad_schema_fragments'']==0"' -f $suite.output,$variant
        $null=Invoke-HostCase $suite "roundtrip_$variant" '' $check -Prerequisite $converted -EnvironmentProblem $converterProblem
        if($variant -eq 'base') { $baseReady=$senderReady }
    }
    $compile='call "{0}" >nul && cl /nologo /TC /std:c11 /D_CRT_SECURE_NO_WARNINGS /Itests/pid_tuning/stubs tests/pid_tuning/test_blackbox_schedule.c /Fo:"{1}/schedule.o" /Fe:"{1}/schedule.exe"' -f $VcVars,$suite.output
    $null=Invoke-HostCase $suite 'schedule' $compile ('"{0}/schedule.exe"' -f $suite.output) -EnvironmentProblem $compilerProblem
    $reserveFlags='/DBB_HOST_TEST /DBB_FILE_BYTES=41943040 /DBB_MAX_FILE_BYTES=67108864 /DBB_DIAGNOSTIC_MS=0 /Itests/pid_tuning/stubs /IApp /IApp/storage /IMiddlewares/Third_Party/FatFs/src'
    $compile='call "{0}" >nul && cl /nologo /std:c11 /D_CRT_SECURE_NO_WARNINGS {1} tests/test_sd_storage.c Middlewares/Third_Party/FatFs/src/ff.c /Fo:"{2}/" /Fe:"{2}/reserve.exe"' -f $VcVars,$reserveFlags,$suite.output
    $null=Invoke-HostCase $suite 'reserve' $compile ('"{0}/reserve.exe" "{0}/sd_base.wire" "{0}/reserve.bin"' -f $suite.output) -Prerequisite $baseReady -EnvironmentProblem $compilerProblem
    $compile='call "{0}" >nul && cl /nologo /TC /std:c11 /D_CRT_SECURE_NO_WARNINGS /Itests/pid_tuning/stubs tests/pid_tuning/test_bmi088_attitude.c /Fo:"{1}/attitude.o" /Fe:"{1}/attitude.exe"' -f $VcVars,$suite.output
    $null=Invoke-HostCase $suite 'attitude' $compile ('"{0}/attitude.exe"' -f $suite.output) -EnvironmentProblem $compilerProblem
} finally {Pop-Location}
exit (Complete-HostSuite $suite)
