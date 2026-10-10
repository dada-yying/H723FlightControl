param([string]$VcVars = 'D:\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
$ErrorActionPreference = 'Stop'
$testOutput = Join-Path ([IO.Path]::GetTempPath()) ('pid-tuning-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testOutput | Out-Null
Push-Location (Join-Path $PSScriptRoot '..\..')
try {
    $crossDir = Join-Path $PSScriptRoot '../../../STM32F407VET6黑匣子/tests/build'
    New-Item -ItemType Directory -Path $crossDir -Force | Out-Null
    $crossFile = Join-Path (Resolve-Path $crossDir) 'h723_wire.bin'
    foreach ($testName in @('test_tuning', 'test_notch', 'test_vofa_esc', 'test_blackbox', 'test_blackbox_schedule', 'test_optical_flow', 'test_optical_diag', 'test_bmi088', 'test_bmi088_heater', 'test_bmi088_attitude')) {
        $fixture = if ($testName -eq 'test_blackbox') { Join-Path (Resolve-Path $crossDir) 'h723_v3_wire.bin' } else { $crossFile }
        $compileCommand = '"{0}" >nul && cl /nologo /TC /std:c11 /D_CRT_SECURE_NO_WARNINGS /I tests\pid_tuning\stubs tests\pid_tuning\{2}.c /Fo:"{1}\{2}.o" /Fe:"{1}\{2}.exe" && "{1}\{2}.exe" "{3}"' -f $VcVars, $testOutput, $testName, $fixture
        & cmd /d /s /c $compileCommand
        if ($LASTEXITCODE -ne 0) { throw "$testName host tests failed." }
    }
    $extra = Join-Path (Resolve-Path $crossDir) 'h723_v3_extra.bin'
    $compileCommand = '"{0}" >nul && cl /nologo /TC /std:c11 /D_CRT_SECURE_NO_WARNINGS /DBLACKBOX_TEST_EXTRA_FIELDS /I tests\pid_tuning\stubs tests\pid_tuning\test_blackbox.c /Fo:"{1}\extra.o" /Fe:"{1}\extra.exe" && "{1}\extra.exe" "{2}"' -f $VcVars, $testOutput, $extra
    & cmd /d /s /c $compileCommand
    if ($LASTEXITCODE -ne 0) { throw 'Blackbox extension test failed' }
} finally { Pop-Location }
