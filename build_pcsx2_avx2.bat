@echo off
echo Building PCSX2 Qt AVX2...

REM Setup Visual Studio environment and build using MSBuild
echo Setting up Visual Studio environment...
call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

echo Building with MSBuild (clean build)...
msbuild "PCSX2_qt.sln" /m /v:m /t:Rebuild /p:Configuration="Release AVX2" /p:Platform="x64"

if %ERRORLEVEL% neq 0 (
    echo Build failed with error %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

echo Build completed successfully!
pause