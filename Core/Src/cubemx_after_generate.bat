@echo off
setlocal
pushd "%~dp0\..\.."
if not exist "Middlewares\Third_Party\FreeRTOS\Source\portable\Common" mkdir "Middlewares\Third_Party\FreeRTOS\Source\portable\Common"
copy /Y "Core\Src\cubemx_mpu_wrappers_v2_stub.c" "Middlewares\Third_Party\FreeRTOS\Source\portable\Common\mpu_wrappers_v2.c" >nul
popd
endlocal
