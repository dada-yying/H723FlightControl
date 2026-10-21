param([switch]$ExpectOldLogic,[string]$VcVars='D:\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    New-Item -ItemType Directory tests/build -Force | Out-Null
    $src=[IO.File]::ReadAllText((Join-Path (Get-Location) 'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c'))
    $start=$src.IndexOf('HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,')
    $end=$src.IndexOf('HAL_StatusTypeDef HAL_SPI_Transmit_IT(',$start)
    if($start -lt 0 -or $end -le $start){throw 'Production HAL function not found'}
    # Generated build fixture, never a tracked source rewrite. Function body is
    # copied from production, not a hand-maintained duplicate implementation.
    [IO.File]::WriteAllText((Join-Path (Get-Location) 'tests/build/spi_poll_body.inc'),$src.Substring($start,$end-$start))
    $def=if($ExpectOldLogic){'/DSD_POLL_EXPECT_OLD'}else{''}
    & cmd /d /s /c ('call "{0}" >nul && cl /nologo /std:c11 {1} /Itests/sd_stubs tests/test_sd_poll_hal.c /Fo:tests/build/sd_poll.o /Fe:tests/build/sd_poll.exe && tests\build\sd_poll.exe' -f $VcVars,$def)
    if($LASTEXITCODE -ne 0){throw 'Actual HAL polling test failed'}
} finally {Pop-Location}
