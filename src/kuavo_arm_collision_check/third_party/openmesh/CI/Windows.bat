mkdir build-release

cd build-release

IF "%ARCHITECTURE%" == "x64" (
  set ARCH_VS= Win64
  set STRING_ARCH=64-Bit
) else (
  set ARCH_VS=
  set STRING_ARCH=32-Bit
)

IF "%SHARED%" == "TRUE" (
  set STRING_DLL=-DLL
) ELSE (
  set STRING_DLL=
)


IF "%BUILD_PLATFORM%" == "VS2015" (
    set LIBPATH=E:\libs\VS2015
    set GENERATOR=Visual Studio 14%ARCH_VS%
    set VS_PATH="C:\Program Files (x86)\Microsoft Visual Studio 14.0\Common7\IDE\devenv.com"

    IF "%ARCHITECTURE%" == "x64" (
      set QT_INSTALL_PATH=E:\Qt\Qt5.6.0-vs2015-%STRING_ARCH%\5.6\msvc2015_64
      set QT_BASE_CONFIG=-DQT_INSTALL_PATH=E:\Qt\Qt5.6.0-vs2015-%STRING_ARCH%\5.6\msvc2015_64 -DQT_VERSION=5
    )

    IF "%ARCHITECTURE%" == "x32" (
      set QT_INSTALL_PATH=E:\Qt\Qt5.6.0-vs2015-%STRING_ARCH%\5.6\msvc2015
      set QT_BASE_CONFIG=-DQT_INSTALL_PATH=E:\Qt\Qt5.6.0-vs2015-%STRING_ARCH%\5.6\msvc2015 -DQT_VERSION=5
    )

) 

IF "%BUILD_PLATFORM%" == "VS2017" (
    set LIBPATH=E:\libs\VS2017
    set GENERATOR=Visual Studio 15%ARCH_VS%
    set VS_PATH="C:\Program Files (x86)\Microsoft Visual Studio\2017\Professional\Common7\IDE\devenv.com"

    IF "%ARCHITECTURE%" == "x64" (
      set QT_INSTALL_PATH=E:\Qt\Qt5.10.1\5.10.1\msvc2017_64
      set QT_BASE_CONFIG=-DQT_INSTALL_PATH=E:\Qt\Qt5.10.1\5.10.1\msvc2017_64 -DQT_VERSION=5
    )

) 


IF "%BUILD_PLATFORM%" == "VS2019" (
    set LIBPATH=E:\libs\VS2019
    set GENERATOR=Visual Studio 16 2019
    set VS_PATH="C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\IDE\devenv.com"

    IF "%ARCHITECTURE%" == "x64" (
      set QT_INSTALL_PATH=E:\Qt\6.1.2\msvc2019_64
      set QT_BASE_CONFIG=-DQT_INSTALL_PATH=E:\Qt\6.1.2\msvc2019_64 -DQT_VERSION=6
    )

)

IF "%APPS%" == "ON" (
  set STRING_APPS=

  ECHO "Copying Platform plugins from %QT_INSTALL_PATH%\plugins\platforms to Build\plugins\platforms"
  

  
  REM Create the platform plugins subdirectory for the qt plugins required to run the gui apps
  mkdir Build
  mkdir Build\plugins
  mkdir Build\plugins\platforms
  
  REM Copy the platform plugins subdirectory for the qt plugins required to run the gui apps
  xcopy /Y %QT_INSTALL_PATH%\plugins\platforms Build\plugins\platforms 
  set CMAKE_CONFIGURATION=%QT_BASE_CONFIG%
) ELSE (
  set STRING_APPS=-no-apps
  set CMAKE_CONFIGURATION=
)




ECHO "============================================================="
ECHO "============================================================="
ECHO "Building with :"
whoami
ECHO "ARCHITECTURE        : %ARCHITECTURE%"
ECHO "BUILD_PLATFORM      : %BUILD_PLATFORM%"
ECHO "GENERATOR           : %GENERATOR%"
ECHO "VS_PATH             : %VS_PATH%"
ECHO "LIBPATH             : %LIBPATH%"
ECHO "APPS                : %APPS%"
ECHO "SHARED              : %SHARED%"
ECHO "QT_INSTALL_PATH     : %QT_INSTALL_PATH%"
ECHO "CMAKE_CONFIGURATION : %CMAKE_CONFIGURATION%"
ECHO "============================================================="
ECHO "============================================================="
ECHO ""
ECHO "Running Build environment checks"

IF EXIST %LIBPATH%\ (
  ECHO "LIBPATH ... Ok"
) ELSE (
  ECHO "LIBPATH not found!"
  exit 10;
)


IF EXIST %QT_INSTALL_PATH%\ (
  ECHO "QT_INSTALL_PATH ... Ok"
) ELSE (
  ECHO "QT_INSTALL_PATH: %QT_INSTALL_PATH%\ not found!"
  exit 10;
)


"C:\Program Files\CMake\bin\cmake.exe" -G "%GENERATOR%"  -DCMAKE_BUILD_TYPE=Release -DBUILD_APPS=%APPS% -DOPENMESH_BUILD_UNIT_TESTS=TRUE -DCMAKE_WINDOWS_LIBS_DIR="e:\libs" -DOPENMESH_BUILD_SHARED=%SHARED% %CMAKE_CONFIGURATION% ..

%VS_PATH% /Build "Release" OpenMesh.sln /Project "ALL_BUILD"

IF %errorlevel% NEQ 0 exit /b %errorlevel%

cd unittests

unittests.exe --gtest_output=xml:./report.xml

unittests_customvec.exe --gtest_output=xml:./report-customvec.xml

unittests_doublevec.exe --gtest_output=xml:./report-doublevec.xml

cd ..

cd ..

mkdir build-debug

cd build-debug

"C:\Program Files\CMake\bin\cmake.exe" -G "%GENERATOR%" -DOPENMESH_BUILD_UNIT_TESTS=TRUE -DCMAKE_BUILD_TYPE=Debug -DOPENMESH_BUILD_SHARED=%SHARED% -DBUILD_APPS=%APPS% %CMAKE_CONFIGURATION% ..

%VS_PATH% /Build "Debug" OpenMesh.sln /Project "ALL_BUILD"

IF %errorlevel% NEQ 0 exit /b %errorlevel%


copy Build\lib\*d.lib ..\build-release\Build\lib

IF "%SHARED%" == "TRUE" (
  copy Build\*.dll ..\build-release\Build
) 


cd unittests

unittests.exe --gtest_output=xml

unittests_customvec.exe --gtest_output=xml

unittests_doublevec.exe --gtest_output=xml

IF %errorlevel% NEQ 0 exit /b %errorlevel%

cd ..

cd ..

cd build-release

del *.exe

"C:\Program Files\CMake\bin\cmake.exe" -G "%GENERATOR%" -DBUILD_APPS=%APPS% -DCMAKE_BUILD_TYPE=Release %CMAKE_CONFIGURATION% ..

%VS_PATH% /Build "Release" OpenMesh.sln /Project "PACKAGE"

IF %errorlevel% NEQ 0 exit /b %errorlevel%

move OpenMesh-*.exe "OpenMesh-11.0-Git-Master-%BUILD_PLATFORM%-%STRING_ARCH%%STRING_DLL%%STRING_APPS%.exe"



