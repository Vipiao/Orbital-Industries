@echo off
echo Setting up Assimp for UCRT64 environment

REM Clean directories
if exist bin\*.dll del /Q bin\*.dll
if exist lib\libassimp.a del /Q lib\libassimp.a
if exist build rmdir /S /Q build
mkdir bin
mkdir build

REM Copy all required DLLs
copy "C:\msys64\ucrt64\bin\libassimp*.dll" bin\
copy "C:\msys64\ucrt64\bin\libgcc_s_seh-1.dll" bin\
copy "C:\msys64\ucrt64\bin\libstdc++-6.dll" bin\
copy "C:\msys64\ucrt64\bin\libwinpthread-1.dll" bin\
copy "C:\msys64\ucrt64\bin\zlib1.dll" bin\
copy "C:\msys64\ucrt64\bin\libminizip-1.dll" bin\
copy "C:\msys64\ucrt64\bin\libbz2-1.dll" bin\
copy "C:\msys64\ucrt64\bin\libzstd.dll" bin\
copy "C:\msys64\ucrt64\bin\libdeflate.dll" bin\
copy "C:\msys64\ucrt64\bin\libpng16-16.dll" bin\
copy "C:\msys64\ucrt64\bin\libssp-0.dll" bin\
copy "C:\msys64\ucrt64\bin\libiconv-2.dll" bin\

echo Copying DLLs completed!
pause