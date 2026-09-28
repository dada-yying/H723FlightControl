param([string]$VcVars = 'D:\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
$ErrorActionPreference = 'Stop'
$testOutput = Join-Path ([IO.Path]::GetTempPath()) ('pid-tuning-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testOutput | Out-Null
Push-Location (Join-Path $PSScriptRoot '..\..')
try {
    $compileCommand = '"{0}" >nul && cl /nologo /TC /std:c11 /I tests\pid_tuning\stubs tests\pid_tuning\test_tuning.c /Fo:"{1}\test.o" /Fe:"{1}\test.exe" && "{1}\test.exe"' -f $VcVars, $testOutput
    & cmd /d /s /c $compileCommand
    if ($LASTEXITCODE -ne 0) { throw 'PID tuning host tests failed.' }
} finally { Pop-Location }
